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
package chisel4ml.tests

import _root_.chisel4ml.tests.SlidingWindowUnitTestBed
import _root_.chisel4ml.util._
import _root_.chisel4ml.sequential._
import _root_.lbir.Datatype.QuantizationType.UNIFORM
import _root_.lbir._
import _root_.services._

import _root_.org.slf4j.LoggerFactory
import chisel3._
import chiseltest._
import org.scalatest.flatspec.AnyFlatSpec

import org.nd4j.linalg.api.ndarray.INDArray
import org.nd4j.linalg.factory.Nd4j
import org.nd4j.linalg.indexing.NDArrayIndex
import org.nd4j.linalg.ops.transforms.Transforms

class ProcessingElementSequentialConvTests extends AnyFlatSpec with ChiselScalatestTester {
  val logger = LoggerFactory.getLogger(classOf[ProcessingElementSequentialConvTests])

  val dtypeUInt4 = lbir.Datatype(quantization = UNIFORM, bitwidth = 4, signed = false, shift = Seq(0), offset = Seq(0))
  val dtypeSInt4 = lbir.Datatype(quantization = UNIFORM, bitwidth = 4, signed = true, shift = Seq(0), offset = Seq(0))

  val testLayer0 = lbir.Layer(
                    ltype  = lbir.Layer.Type.CONV2D,
                    thresh = Option(lbir.QTensor(
                              dtype = Option(dtypeSInt4),
                              shape = Seq(1),
                              values = Seq(0)
                             )),
                    weights = Option(lbir.QTensor(
                                dtype = Option(dtypeSInt4),
                                shape = Seq(1, 1, 2, 2),
                                values = Seq(1, 0,
                                             0, 0)
                              )),
                    input = Option(lbir.QTensor(
                              dtype = Option(dtypeUInt4),
                              shape = Seq(1, 1, 3, 3)
                            )),
                    output = Option(lbir.QTensor(
                              dtype = Option(dtypeSInt4),
                              shape = Seq(1, 1, 2, 2)
                            )),
                    activation = lbir.Layer.Activation.NO_ACTIVATION
                   )

  val testOptions0 = services.GenerateCircuitParams.Options(isSimple = false)


  val dtypeUInt6 = lbir.Datatype(quantization = UNIFORM, bitwidth = 6, signed = false, shift = Seq(0), offset = Seq(0))
  val dtypeSInt7 = lbir.Datatype(quantization = UNIFORM, bitwidth = 7, signed = true, shift = Seq(0), offset = Seq(0))
  val testLayer1 = lbir.Layer(
                    ltype = lbir.Layer.Type.CONV2D,
                    thresh = Option(lbir.QTensor(
                      dtype = Option(dtypeSInt7),
                      shape = Seq(2),
                      values = Seq(-2)
                    )),
                    weights = Option(lbir.QTensor(
                      dtype = Option(dtypeSInt7),
                      shape = Seq(1, 2, 2, 2),
                      values = Seq(1, 0,
                                   0, 0,
                                   0, 0,
                                   1, 0)
                    )),
                    input = Option(lbir.QTensor(
                      dtype = Option(dtypeUInt6),
                      shape = Seq(1, 2, 3, 3),
                    )),
                    output = Option(lbir.QTensor(
                      dtype = Option(dtypeSInt7),
                      shape = Seq(1, 1, 2, 2)
                    ))
                   )


  behavior.of("ProcessingElementSequentialConv module")
  it should "compute the convolution correctly" in { // .withAnnotations(Seq(VerilatorBackendAnnotation))
    test(new ProcessingElementSequentialConv[UInt, SInt, SInt, SInt, SInt](layer = testLayer0,
                                                                           options = testOptions0,
                                                                           genIn = UInt(4.W),
                                                                           genWeights = SInt(4.W),
                                                                           genThresh = SInt(4.W),
                                                                           genOut = SInt(4.W),
                                                                           mul = (x:UInt, w: SInt) => (x * w),
                                                                           add = (x: Vec[SInt]) => x.reduceTree(_ +& _),
                                                                           actFn = (x: SInt, y: SInt) => x)).
                                                                           withAnnotations(
                                                                           Seq(VerilatorBackendAnnotation)) { dut =>
      dut.io.inStream.data.initSource()
      dut.io.inStream.data.setSourceClock(dut.clock)
      dut.io.outStream.data.initSink()
      dut.io.outStream.data.setSinkClock(dut.clock)
      dut.clock.step(1)
      dut.io.inStream.data.enqueueSeq(Seq("b1000_0111__0110_0101__0100_0011__0010_0001".U,
                                          "b0000_0000__0000_0000__0000_0000__0000_1001".U))
      dut.io.inStream.last.poke(true.B)
      dut.clock.step(1)
      dut.io.inStream.last.poke(false.B)
      dut.io.outStream.data.expectDequeueSeq(Seq("b0000_0000__0000_0000__0101_0100__0010_0001".U))
    }
  }

  it should "compute a convolution with several channels correctly" in {
    test(new ProcessingElementSequentialConv[UInt, SInt, SInt, SInt, SInt](layer = testLayer1,
                                                                           options = testOptions0,
                                                                           genIn = UInt(6.W),
                                                                           genWeights = SInt(7.W),
                                                                           genThresh = SInt(7.W),
                                                                           genOut = SInt(7.W),
                                                                           mul = (x:UInt, w: SInt) => (x * w),
                                                                           add = (x: Vec[SInt]) => x.reduceTree(_ +& _),
                                                                           actFn = (x: SInt, y:SInt) => x + y)).
                                                                           withAnnotations(
                                                                           Seq(VerilatorBackendAnnotation)){ dut =>
      dut.clock.setTimeout(10000)
      dut.io.inStream.data.initSource()
      dut.io.inStream.data.setSourceClock(dut.clock)
      dut.io.outStream.data.initSink()
      dut.io.outStream.data.setSinkClock(dut.clock)
      dut.clock.step()
      /*  1   2   3   |   1  0  |  1 + 13 - 2 = 12  | 12 14
       *  4   5   6   |   0  0  |  2 + 14 - 2 = 14  | 18 20
       *  7   8   9   |         |  4 + 16 - 2 = 18  |
       *              |         |  5 + 17 - 2 = 20  |
       *  10 11  12   |   0  0  |                   |
       *  13 14  15   |   1  0  |                   |
       *  16 17  18   |         |                   |
       */

      dut.io.inStream.data.enqueueSeq(Seq("b00_000101_000100_000011_000010_000001".U,
                                          "b00_001010_001001_001000_000111_000110".U,
                                          "b00_001111_001110_001101_001100_001011".U,
                                          "b00_000000_000000_010010_010001_010000".U))
      dut.io.inStream.last.poke(true.B)
      dut.clock.step()
      dut.io.inStream.last.poke(false.B)
      dut.io.outStream.data.expectDequeueSeq(Seq("b0000_0010100_0010010_0001110_0001100".U))
    }
  }
}
