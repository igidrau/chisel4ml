/*
 * HEADER: TODO
 *
 */
package chisel4ml

import _root_.java.nio.file.{Files, Path, Paths}
import _root_.java.util.concurrent.TimeUnit
import _root_.scala.concurrent.{ExecutionContext, Future}

import _root_.chisel3.stage._
import _root_.chisel3._
import _root_.firrtl.stage.FirrtlCircuitAnnotation  
import _root_.firrtl.{AnnotationSeq, EmittedCircuitAnnotation}                                                                 
import _root_.firrtl.annotations.{Annotation, DeletedAnnotation}  

import _root_.io.grpc.{Server, ServerBuilder}
import _root_.services._
import _root_.services.GenerateCircuitReturn.ErrorMsg
import _root_.lbir.{Datatype, Model, QTensor}
import _root_.chisel4ml.util.LbirUtil
import _root_.chisel4ml.Circuit

import _root_.chiseltest.simulator._

import _root_.treadle.TreadleTester
import _root_.org.slf4j.Logger
import _root_.org.slf4j.LoggerFactory

/** An object extending App to generate the Verilog code.
  */
object Chisel4mlServer {
    private val port = 50051
   
    def main(args: Array[String]): Unit = {
        require(args.length > 0, "No argument list, you should provide an argument as a directory.")
        require(Files.exists(Paths.get(args(0))), "Provided directory doesn't exist.")
        val server = new Chisel4mlServer(ExecutionContext.global, tempDir=args(0))
        server.start()
        server.blockUntilShutdown()
    }
}

class Chisel4mlServer(executionContext: ExecutionContext, tempDir: String) { self =>
    private[this] var server: Server = null
    private var circuits: Seq[Circuit] = Seq() // Holds the circuit and simulation object
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
            circuits.map(_.stopSimulation())
            logger.info("Shutting down chisel4ml server.")
            server.shutdown()
        } else { logger.error("Attempted to shut down server that was not created.") }
    }

    private def blockUntilShutdown(): Unit = { if (server != null) { server.awaitTermination() } }

    private object Chisel4mlServiceImpl extends Chisel4mlServiceGrpc.Chisel4mlService {
        override def generateCircuit(params: GenerateCircuitParams): Future[GenerateCircuitReturn] = {
            val circuitId = circuits.length 
            circuits = circuits :+ new Circuit(model = params.model.get, 
                                               options = params.options.get, 
                                               directory = Path.of(tempDir, s"circuit$circuitId"), 
                                               useVerilator = params.useVerilator,
                                               genVcd = params.genVcd)
            logger.info(s"Started generating hardware for circuit id:$circuitId in temporary directory.")
            new Thread(circuits.last).start()
            if (circuits.last.isGenerated.await(params.generationTimeoutSec, TimeUnit.SECONDS)) {
                logger.info("Succesfully generated circuit.")
                Future.successful(GenerateCircuitReturn(circuitId=circuits.length-1, 
                                                        err=Option(ErrorMsg(errId = ErrorMsg.ErrorId.SUCCESS, 
                                                                   msg = "Successfully generated verilog."))))
            } else {
                logger.error("Circuit generation timed-out, please try again with a longer timeout.")
                Future.successful(GenerateCircuitReturn(circuitId=circuits.length-1, 
                                                        err=Option(ErrorMsg(errId = ErrorMsg.ErrorId.FAIL, 
                                                                   msg = "Error generating circuit."))))
            }
        }

        override def runSimulation(params: RunSimulationParams): Future[RunSimulationReturn] = {
            logger.info(s"Simulating circuit id: ${params.circuitId} circuit on ${params.inputs.length} input/s.")
            Future.successful(
                RunSimulationReturn(values=Seq(circuits(params.circuitId).sim(params.inputs(0))))
            )
        }
    }
}
