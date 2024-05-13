package chisel4ml

import chisel4ml.implicits._
import lbir.DenseConfig
import chisel3._
import chisel3.util._
import interfaces.amba.axis._
import org.chipsalliance.cde.config.{Field, Parameters}
import chisel4ml.logging.HasParameterLogging

case object DenseConfigField extends Field[DenseConfig]

trait HasDenseParameters extends HasLBIRStreamParameters[DenseConfig] {
  type T = DenseConfig
  val p: Parameters
  val cfg = p(DenseConfigField)
}


// Simple implementation of a FIFO buffer
class FIFO[T <: Data](size) {
  val fifo = Reg(Vec(size, T))
  val firstFree = RegInit(0.U)
  val lastFree = RegInit((size - 1).U)

  // Check whether the FIFO has enough free space for length elements
  def isFree(length) {
    val freeSpace = (lastFree - firstFree + 1) + (
      when(firstFree > lastFree + 1) {size}.otherwise {0}
    )
    return (freeSpace >= length)
  }

  // Pushes a vector to the end of the FIFO
  def push(input : Vec[T]) {
    for (i <- 0 until input.length) {
      fifo((firstFree + i) % size) = input(i)
    }
    firstFree += input.length
    firstFree = firstFree % size
  }

  // Check whether the FIFO contains at least length elements
  def isOccupied(length) {
    val occupiedSpace = (lastFree - firstFree + 1) + (
      when(firstFree > lastFree + 1) {size}.otherwise {0}
    )
    val occupiedSpace =  mod((firstFree - 1 - lastFree), size)
    return (occupiedSpace >= length)
  }

  // Returns length elements from the front of the FIFO and advance as much
  def pop(length) {
    val retval = Vec(length, T)
    for (i <- 0 until length) {
      retval(i) = fifo((lastFree + 1 + i) % size)
    }
    lastFree += length
    lastFree = lastFree % size
    return retval
  }

  // Returns length elements from the front of the FIFO, without advancing
  def read(length) {
    val retval = Vec(length, T)
    for (i <- 0 until length) {
      retval(i) = fifo((lastFree + 1 + i) % size)
    }
    return retval
  }
}

object ProcessingElementWrapDynamicToSequential {
  object State extends ChiselEnum {
    val sWaitIn, sWaitWeights, sCompute = Value
  }
}


class ProcessingElementWrapDynamicToSequential(implicit val p: Parameters)
    extends Module
    with HasLBIRStream[Vec[UInt]]
    with HasLBIRStreamParameters[DenseConfig]
    with HasDenseParameters
    with HasParameterLogging {
  logParameters
  val inStream = IO(Flipped(AXIStream(
    Vec(numBeatsIn, UInt(cfg.input.dtype.bitwidth.W))
  )))
  val weightStream = IO(Flipped(AXIStream(
    Vec(numBeatsWeight, Uint(cfg.kernel.dtype.bitwidth.W))
  )))
  val biasStream = IO(Flipped(AXIStream(
    Vec(numBeatsBias, Uint(cfg.thresh.dtype.bitwidth.W))
  )))

  val outStream = IO(AXIStream(
    Vec(numBeatsOut, UInt(cfg.output.dtype.bitwidth.W))
  ))

  val inBuff = new FIFO[Vec(numBeatsIn, UInt(cfg.input.dtype.bitwidth.W))](
    cfg.input.shape(0))
  val weightBuff = new FIFO[Vec(numBeatsIn, UInt(cfg.input.dtype.bitwidth.W))](
    cfg.kernel.shape(0) * cfg.parallel_lines)
  val biasBuff = new FIFO[Vec(numBeatsIn, UInt(cfg.input.dtype.bitwidth.W))](
    cfg.thresh.shape(0))
  val outBuf = new FIFO[Vec(numBeatsIn, UInt(cfg.input.dtype.bitwidth.W))](
    cfg.output.shape(0))


  val peDynamic = Module(ProcessingElementDynamic(cfg))
  val state = RegInit(sWaitIn)

  /***** INPUT DATA INTERFACE *****/
  inStream.ready := inBuff.isFree(numBeatsIn).B
  when(inStream.fire) {
    inBuff.push(inStream.bits)
  }
  weightStream.ready := weightBuff.isFree(numBeatsWeight).B
  when(inStream.fire) {
    weightBuff.push(weightStream.bits)
  }
  biasStream.ready := biasBuff.isFree(numBeatsBias).B
  when(inStream.fire) {
    biasBuff.push(biasStream.bits)
  }

  switch (state) {
    is (sWaitIn) {
      computed_lines := 0.U
      when (inStream.isOccupied(cfg.input.shape(0))) {
        peDynamic.in := inBuff.pop(cfg.input.shape(0))
        state := sWaitWeights
      }
    }
    is (sWaitWeights) {
      when (weightStream.isOccupied(cfg.kernel.shape(0) * cfg.parallel_lines) &&
        biasStream.isOccupied(cfg.thresh.shape(0))
      ) {
        peDynamic.weight := weightBuff.pop(cfg.kernel.shape(0) * cfg.parallel_lines)
        peDynamic.thresh := biasBuff.pop(cfg.thresh.shape(0))
        computed_lines := computed_lines + cfg.parallel_lines
        state := sCompute
      }
    }
    is (sCompute) {
      when (outStream.isFree(cfg.parallel_lines)) {
        outStream.push(peDynamic.out)
        when (computed_lines >= cfg.kernel.shape(1)) {
          state := sWaitIn
        }.otherwise {
          state := sWaitWeights
        }
      }
    }
  }



  /***** OUTPUT DATA INTERFACE *****/
  outStream.valid := outBuff.isOccupied(numBeatsOut)
  outStream.bits := outBuff.read(numBeatsOut)
  when(outStream.fire) {
    weightBuff.pop(numBeatsOut)
  }
  outStream.last := outputCntWrap
}
