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

  val (inputCntValue, inputCntWrap) = Counter(inStream.fire, cfg.input.numTransactions(inWidth))
  val (weightCntValue, weightCntWrap) = Counter(weightStream.fire, cfg.kernel.numTransactions(weightWidth))
  val (biasCntValue, biasCntWrap) = Counter(biasStream.fire, cfg.thresh.numTransactions(biasWidth))
  val (outputCntValue, outputCntWrap) = Counter(outStream.fire, cfg.output.numTransactions(outWidth))
  val outputBufferFull = RegInit(false.B)



  /** *** INPUT DATA INTERFACE ****
    */
  inStream.ready := !outputBufferFull
  when(inStream.fire) {
    inputBuffer(inputCntValue) := inStream.bits.asUInt
  }


  /** *** OUTPUT DATA INTERFACE ****
    */
  outStream.valid := outputBufferFull
  outStream.bits := outputBuffer(outputCntValue).asTypeOf(outStream.bits)
  outStream.last := outputCntWrap
}
