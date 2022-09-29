/*
 * HEADER: TODO
 *
 */
package chisel4ml

import _root_.java.nio.file.{Files, Paths}
import _root_.scala.concurrent.{ExecutionContext, Future}

import _root_.chisel3.stage._
import _root_.chisel3._

import _root_.io.grpc.{Server, ServerBuilder}
import _root_.services._
import _root_.services.GenerateCircuitReturn.ErrorMsg
import _root_.lbir.{Datatype, Model, QTensor}
import _root_.chisel4ml.util.LbirUtil
import _root_.chisel4ml.Circuit

import _root_.treadle.TreadleTester
import _root_.org.slf4j.Logger
import _root_.org.slf4j.LoggerFactory

/** An object extending App to generate the Verilog code.
  */
object Chisel4mlServer {
    private val port = 50051

    def main(args: Array[String]): Unit = {
        val server = new Chisel4mlServer(ExecutionContext.global)
        server.start()
        server.blockUntilShutdown()
    }
}

class Chisel4mlServer(executionContext: ExecutionContext) { self =>
    private[this] var server: Server = null
    val logger = LoggerFactory.getLogger(classOf[Chisel4mlServer])

    private def start(): Unit = {
        server = ServerBuilder
            .forPort(Chisel4mlServer.port)
            .addService(Chisel4mlServiceGrpc.bindService(Chisel4mlServiceImpl, executionContext))
            .build
            .start
        sys.addShutdownHook { self.stop() }
        logger.info("Started a new chisel4ml server.")
    }

    private def stop(): Unit = {
        if (server != null) {
            logger.info("Shutting down chisel4ml server.")
            server.shutdown()
        } else { logger.error("Attempted to shut down server that was not created.") }
    }

    private def blockUntilShutdown(): Unit = { if (server != null) { server.awaitTermination() } }

    private object Chisel4mlServiceImpl extends Chisel4mlServiceGrpc.Chisel4mlService {
        private var circuits: Seq[Circuit] = Seq()

        override def generateCircuit(params: GenerateCircuitParams): Future[GenerateCircuitReturn] = {
            val model = (new ChiselStage).execute(Array("--target-dir", params.directory),
              Seq(ChiselGeneratorAnnotation(() => new ProcessingPipelineSimple(params.model.get)))
            )
            val output = params.model.get.layers.last.output.get
            val tester = TreadleTester(model)
            circuits = circuits :+ Circuit(model=model, output=output, tester=tester)
            logger.info(s"Generating hardware for circuit id:${circuits.length-1} in directory:${params.directory} .")
            Future.successful(GenerateCircuitReturn(circuitId=circuits.length-1, 
                                                    err=Option(ErrorMsg(errId = ErrorMsg.ErrorId.SUCCESS, 
                                                                        msg = "Successfully generated verilog."))))
        }

        override def runSimulation(params: RunSimulationParams): Future[RunSimulationReturn] = {
            logger.info(s"Simulating circuit id: ${params.circuitId} circuit on ${params.inputs.length} input/s.")
            circuits(params.circuitId).tester.poke("io_in", LbirUtil.qtensorToBigInt(params.inputs(0)))
            Future.successful(RunSimulationReturn(values = List(
                                LbirUtil.bigIntToQtensor(circuits(params.circuitId).tester.peek("io_out"), 
                                circuits(params.circuitId).output))))
        }
    }
}
