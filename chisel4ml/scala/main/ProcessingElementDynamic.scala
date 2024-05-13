/*
 * Copyright 2022 Computer Systems Department, Jozef Stefan Insitute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package chisel4ml

import chisel4ml.{NeuronWithBias, NeuronWithoutBias}
import chisel4ml.implicits._
import chisel4ml.quantization._
import chisel4ml._
import lbir.Datatype.QuantizationType._
import lbir.DenseConfig
import chisel3._
import spire.algebra.Ring
import spire.implicits._
import dsptools.numbers._

object ProcessingElementDynamic {
  def apply(layer: DenseConfig) = (
    layer.input.dtype.quantization,
    layer.input.dtype.signed,
    layer.kernel.dtype.quantization,
    layer.output.dtype.signed
  ) match {
    case (UNIFORM, true, UNIFORM, false) =>
      new ProcessingElementDynamic[SInt, SInt, SInt, SInt, UInt](layer)(
        new UniformQuantizationContextSSUReLU(layer.roundingMode)
      )
    case (UNIFORM, false, UNIFORM, false) =>
      new ProcessingElementDynamic[UInt, SInt, SInt, SInt, UInt](layer)(
        new UniformQuantizationContextUSUReLU(layer.roundingMode)
      )
    case (UNIFORM, true, UNIFORM, true) =>
      new ProcessingElementDynamic[SInt, SInt, SInt, SInt, SInt](layer)(
        new UniformQuantizationContextSSSNoAct(layer.roundingMode)
      )
    case (UNIFORM, false, UNIFORM, true) =>
      new ProcessingElementDynamic[UInt, SInt, SInt, SInt, SInt](layer)(
        new UniformQuantizationContextUSSNoAct(layer.roundingMode)
      )
    case (UNIFORM, false, BINARY, true) =>
      new ProcessingElementDynamic[UInt, Bool, SInt, SInt, Bool](layer)(
        new BinaryQuantizationContext(layer.roundingMode)
      )
    case (UNIFORM, true, BINARY, true) =>
      new ProcessingElementDynamic[SInt, Bool, SInt, SInt, Bool](layer)(
        new BinaryQuantizationContextSInt(layer.roundingMode)
      )
    case (BINARY, _, BINARY, true) =>
      new ProcessingElementDynamic[Bool, Bool, Bool, UInt, Bool](layer)(BinarizedQuantizationContext)
    case _ => throw new RuntimeException()
  }
}

class ProcessingElementDynamic[I <: Bits, W <: Bits, M <: Bits, A <: Bits: Ring, O <: Bits](
  layer: DenseConfig
)(qc:    QuantizationContext[I, W, M, A, O])
    extends Module
    with LBIRStreamSimple {
  val in = IO(Input(Vec(layer.input.width, layer.input.getType[I])))
  val weights = IO(Input(
    Vec(layer.output.width,
      Vec(layer.input.width, layer.weights.getType[W]))
  ))
  val thresh = IO(Input(Vec(layer.output.width, layer.thresh.getType[A])))
  val out = IO(Output(Vec(layer.output.width, layer.output.getType[O])))

  val shift:   Seq[Int] = layer.kernel.dtype.shift
  val neurons = Seq[DynamicNeuron]()

  for (i <- 0 until layer.output.shape(0)) {
    val muls = VecInit(in.zip(weights(i)).map { case (i, w) => qc.mul(i, w) })
    val threshAdjusted = (thresh << shift.abs).asSInt.asInstanceOf[A]
    val pAct = qc.add(muls) - threshAdjusted
    val sAct = qc.shiftAndRoundStatic(pAct, shift)
    out(i) := qc.actFn(sAct, Ring[A].zero, outputBitwidth)
  }
}
