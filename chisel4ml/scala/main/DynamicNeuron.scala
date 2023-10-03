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

import chisel3._
import chisel4ml.util.{saturate, shiftAndRoundDynamic}
import chisel4ml.conv2d.KernelThreshIO
import chisel4ml.implicits._

class DynamicNeuron[I <: Bits with Num[I], W <: Bits with Num[W], M <: Bits, S <: Bits, A <: Bits, O <: Bits](
  kernel:     lbir.QTensor,
  genIn:      I,
  genWeights: W,
  genAccu:    S,
  genThresh:  A,
  genOut:     O,
  mul:        (I, W) => M,
  add:        Vec[M] => S,
  actFn:      (S, A) => O)
    extends Module {
  val io = IO(new Bundle {
    val in: UInt = Input(UInt((kernel.numKernelParams * genIn.getWidth).W))
    val weights = Flipped(new KernelThreshIO(kernel, genThresh))
    val out: O = Output(genOut)
  })

  val inVec = io.in.asTypeOf(Vec(kernel.numKernelParams, genIn))
  val inWeights = io.weights.kernel.asTypeOf(Vec(kernel.numKernelParams, genWeights))

  val muls = VecInit((inVec.zip(inWeights)).map { case (a, b) => mul(a, b) })
  val pAct = add(muls)
  val sAct = shiftAndRoundDynamic(pAct, io.weights.thresh.shift, io.weights.thresh.shiftLeft, genAccu)
  io.out := saturate(actFn(sAct, io.weights.thresh.thresh).asUInt, genOut.getWidth).asTypeOf(io.out)
}
