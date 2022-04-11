// Generated by the Scala Plugin for the Protocol Buffer Compiler.
// Do not edit!
//
// Protofile syntax: PROTO3

package lbir

@SerialVersionUID(0L)
final case class Layer(
    layerType: lbir.Layer.LayerType = lbir.Layer.LayerType.DENSE,
    useBias: _root_.scala.Boolean = false,
    weights: _root_.scala.Option[lbir.QTensor] = _root_.scala.None,
    biases: _root_.scala.Option[lbir.QTensor] = _root_.scala.None,
    inputQuantizer: _root_.scala.Option[lbir.Quantizer] = _root_.scala.None,
    width: _root_.scala.Int = 0,
    height: _root_.scala.Int = 0,
    channels: _root_.scala.Int = 0,
    unknownFields: _root_.scalapb.UnknownFieldSet = _root_.scalapb.UnknownFieldSet.empty
    ) extends scalapb.GeneratedMessage with scalapb.lenses.Updatable[Layer] {
    @transient
    private[this] var __serializedSizeCachedValue: _root_.scala.Int = 0
    private[this] def __computeSerializedValue(): _root_.scala.Int = {
      var __size = 0
      
      {
        val __value = layerType.value
        if (__value != 0) {
          __size += _root_.com.google.protobuf.CodedOutputStream.computeEnumSize(1, __value)
        }
      };
      
      {
        val __value = useBias
        if (__value != false) {
          __size += _root_.com.google.protobuf.CodedOutputStream.computeBoolSize(2, __value)
        }
      };
      if (weights.isDefined) {
        val __value = weights.get
        __size += 1 + _root_.com.google.protobuf.CodedOutputStream.computeUInt32SizeNoTag(__value.serializedSize) + __value.serializedSize
      };
      if (biases.isDefined) {
        val __value = biases.get
        __size += 1 + _root_.com.google.protobuf.CodedOutputStream.computeUInt32SizeNoTag(__value.serializedSize) + __value.serializedSize
      };
      if (inputQuantizer.isDefined) {
        val __value = inputQuantizer.get
        __size += 1 + _root_.com.google.protobuf.CodedOutputStream.computeUInt32SizeNoTag(__value.serializedSize) + __value.serializedSize
      };
      
      {
        val __value = width
        if (__value != 0) {
          __size += _root_.com.google.protobuf.CodedOutputStream.computeUInt32Size(6, __value)
        }
      };
      
      {
        val __value = height
        if (__value != 0) {
          __size += _root_.com.google.protobuf.CodedOutputStream.computeUInt32Size(7, __value)
        }
      };
      
      {
        val __value = channels
        if (__value != 0) {
          __size += _root_.com.google.protobuf.CodedOutputStream.computeUInt32Size(8, __value)
        }
      };
      __size += unknownFields.serializedSize
      __size
    }
    override def serializedSize: _root_.scala.Int = {
      var read = __serializedSizeCachedValue
      if (read == 0) {
        read = __computeSerializedValue()
        __serializedSizeCachedValue = read
      }
      read
    }
    def writeTo(`_output__`: _root_.com.google.protobuf.CodedOutputStream): _root_.scala.Unit = {
      {
        val __v = layerType.value
        if (__v != 0) {
          _output__.writeEnum(1, __v)
        }
      };
      {
        val __v = useBias
        if (__v != false) {
          _output__.writeBool(2, __v)
        }
      };
      weights.foreach { __v =>
        val __m = __v
        _output__.writeTag(3, 2)
        _output__.writeUInt32NoTag(__m.serializedSize)
        __m.writeTo(_output__)
      };
      biases.foreach { __v =>
        val __m = __v
        _output__.writeTag(4, 2)
        _output__.writeUInt32NoTag(__m.serializedSize)
        __m.writeTo(_output__)
      };
      inputQuantizer.foreach { __v =>
        val __m = __v
        _output__.writeTag(5, 2)
        _output__.writeUInt32NoTag(__m.serializedSize)
        __m.writeTo(_output__)
      };
      {
        val __v = width
        if (__v != 0) {
          _output__.writeUInt32(6, __v)
        }
      };
      {
        val __v = height
        if (__v != 0) {
          _output__.writeUInt32(7, __v)
        }
      };
      {
        val __v = channels
        if (__v != 0) {
          _output__.writeUInt32(8, __v)
        }
      };
      unknownFields.writeTo(_output__)
    }
    def withLayerType(__v: lbir.Layer.LayerType): Layer = copy(layerType = __v)
    def withUseBias(__v: _root_.scala.Boolean): Layer = copy(useBias = __v)
    def getWeights: lbir.QTensor = weights.getOrElse(lbir.QTensor.defaultInstance)
    def clearWeights: Layer = copy(weights = _root_.scala.None)
    def withWeights(__v: lbir.QTensor): Layer = copy(weights = Option(__v))
    def getBiases: lbir.QTensor = biases.getOrElse(lbir.QTensor.defaultInstance)
    def clearBiases: Layer = copy(biases = _root_.scala.None)
    def withBiases(__v: lbir.QTensor): Layer = copy(biases = Option(__v))
    def getInputQuantizer: lbir.Quantizer = inputQuantizer.getOrElse(lbir.Quantizer.defaultInstance)
    def clearInputQuantizer: Layer = copy(inputQuantizer = _root_.scala.None)
    def withInputQuantizer(__v: lbir.Quantizer): Layer = copy(inputQuantizer = Option(__v))
    def withWidth(__v: _root_.scala.Int): Layer = copy(width = __v)
    def withHeight(__v: _root_.scala.Int): Layer = copy(height = __v)
    def withChannels(__v: _root_.scala.Int): Layer = copy(channels = __v)
    def withUnknownFields(__v: _root_.scalapb.UnknownFieldSet) = copy(unknownFields = __v)
    def discardUnknownFields = copy(unknownFields = _root_.scalapb.UnknownFieldSet.empty)
    def getFieldByNumber(__fieldNumber: _root_.scala.Int): _root_.scala.Any = {
      (__fieldNumber: @_root_.scala.unchecked) match {
        case 1 => {
          val __t = layerType.javaValueDescriptor
          if (__t.getNumber() != 0) __t else null
        }
        case 2 => {
          val __t = useBias
          if (__t != false) __t else null
        }
        case 3 => weights.orNull
        case 4 => biases.orNull
        case 5 => inputQuantizer.orNull
        case 6 => {
          val __t = width
          if (__t != 0) __t else null
        }
        case 7 => {
          val __t = height
          if (__t != 0) __t else null
        }
        case 8 => {
          val __t = channels
          if (__t != 0) __t else null
        }
      }
    }
    def getField(__field: _root_.scalapb.descriptors.FieldDescriptor): _root_.scalapb.descriptors.PValue = {
      _root_.scala.Predef.require(__field.containingMessage eq companion.scalaDescriptor)
      (__field.number: @_root_.scala.unchecked) match {
        case 1 => _root_.scalapb.descriptors.PEnum(layerType.scalaValueDescriptor)
        case 2 => _root_.scalapb.descriptors.PBoolean(useBias)
        case 3 => weights.map(_.toPMessage).getOrElse(_root_.scalapb.descriptors.PEmpty)
        case 4 => biases.map(_.toPMessage).getOrElse(_root_.scalapb.descriptors.PEmpty)
        case 5 => inputQuantizer.map(_.toPMessage).getOrElse(_root_.scalapb.descriptors.PEmpty)
        case 6 => _root_.scalapb.descriptors.PInt(width)
        case 7 => _root_.scalapb.descriptors.PInt(height)
        case 8 => _root_.scalapb.descriptors.PInt(channels)
      }
    }
    def toProtoString: _root_.scala.Predef.String = _root_.scalapb.TextFormat.printToUnicodeString(this)
    def companion = lbir.Layer
    // @@protoc_insertion_point(GeneratedMessage[chisel4ml.Layer])
}

object Layer extends scalapb.GeneratedMessageCompanion[lbir.Layer] {
  implicit def messageCompanion: scalapb.GeneratedMessageCompanion[lbir.Layer] = this
  def parseFrom(`_input__`: _root_.com.google.protobuf.CodedInputStream): lbir.Layer = {
    var __layerType: lbir.Layer.LayerType = lbir.Layer.LayerType.DENSE
    var __useBias: _root_.scala.Boolean = false
    var __weights: _root_.scala.Option[lbir.QTensor] = _root_.scala.None
    var __biases: _root_.scala.Option[lbir.QTensor] = _root_.scala.None
    var __inputQuantizer: _root_.scala.Option[lbir.Quantizer] = _root_.scala.None
    var __width: _root_.scala.Int = 0
    var __height: _root_.scala.Int = 0
    var __channels: _root_.scala.Int = 0
    var `_unknownFields__`: _root_.scalapb.UnknownFieldSet.Builder = null
    var _done__ = false
    while (!_done__) {
      val _tag__ = _input__.readTag()
      _tag__ match {
        case 0 => _done__ = true
        case 8 =>
          __layerType = lbir.Layer.LayerType.fromValue(_input__.readEnum())
        case 16 =>
          __useBias = _input__.readBool()
        case 26 =>
          __weights = Option(__weights.fold(_root_.scalapb.LiteParser.readMessage[lbir.QTensor](_input__))(_root_.scalapb.LiteParser.readMessage(_input__, _)))
        case 34 =>
          __biases = Option(__biases.fold(_root_.scalapb.LiteParser.readMessage[lbir.QTensor](_input__))(_root_.scalapb.LiteParser.readMessage(_input__, _)))
        case 42 =>
          __inputQuantizer = Option(__inputQuantizer.fold(_root_.scalapb.LiteParser.readMessage[lbir.Quantizer](_input__))(_root_.scalapb.LiteParser.readMessage(_input__, _)))
        case 48 =>
          __width = _input__.readUInt32()
        case 56 =>
          __height = _input__.readUInt32()
        case 64 =>
          __channels = _input__.readUInt32()
        case tag =>
          if (_unknownFields__ == null) {
            _unknownFields__ = new _root_.scalapb.UnknownFieldSet.Builder()
          }
          _unknownFields__.parseField(tag, _input__)
      }
    }
    lbir.Layer(
        layerType = __layerType,
        useBias = __useBias,
        weights = __weights,
        biases = __biases,
        inputQuantizer = __inputQuantizer,
        width = __width,
        height = __height,
        channels = __channels,
        unknownFields = if (_unknownFields__ == null) _root_.scalapb.UnknownFieldSet.empty else _unknownFields__.result()
    )
  }
  implicit def messageReads: _root_.scalapb.descriptors.Reads[lbir.Layer] = _root_.scalapb.descriptors.Reads{
    case _root_.scalapb.descriptors.PMessage(__fieldsMap) =>
      _root_.scala.Predef.require(__fieldsMap.keys.forall(_.containingMessage eq scalaDescriptor), "FieldDescriptor does not match message type.")
      lbir.Layer(
        layerType = lbir.Layer.LayerType.fromValue(__fieldsMap.get(scalaDescriptor.findFieldByNumber(1).get).map(_.as[_root_.scalapb.descriptors.EnumValueDescriptor]).getOrElse(lbir.Layer.LayerType.DENSE.scalaValueDescriptor).number),
        useBias = __fieldsMap.get(scalaDescriptor.findFieldByNumber(2).get).map(_.as[_root_.scala.Boolean]).getOrElse(false),
        weights = __fieldsMap.get(scalaDescriptor.findFieldByNumber(3).get).flatMap(_.as[_root_.scala.Option[lbir.QTensor]]),
        biases = __fieldsMap.get(scalaDescriptor.findFieldByNumber(4).get).flatMap(_.as[_root_.scala.Option[lbir.QTensor]]),
        inputQuantizer = __fieldsMap.get(scalaDescriptor.findFieldByNumber(5).get).flatMap(_.as[_root_.scala.Option[lbir.Quantizer]]),
        width = __fieldsMap.get(scalaDescriptor.findFieldByNumber(6).get).map(_.as[_root_.scala.Int]).getOrElse(0),
        height = __fieldsMap.get(scalaDescriptor.findFieldByNumber(7).get).map(_.as[_root_.scala.Int]).getOrElse(0),
        channels = __fieldsMap.get(scalaDescriptor.findFieldByNumber(8).get).map(_.as[_root_.scala.Int]).getOrElse(0)
      )
    case _ => throw new RuntimeException("Expected PMessage")
  }
  def javaDescriptor: _root_.com.google.protobuf.Descriptors.Descriptor = LbirProto.javaDescriptor.getMessageTypes().get(1)
  def scalaDescriptor: _root_.scalapb.descriptors.Descriptor = LbirProto.scalaDescriptor.messages(1)
  def messageCompanionForFieldNumber(__number: _root_.scala.Int): _root_.scalapb.GeneratedMessageCompanion[_] = {
    var __out: _root_.scalapb.GeneratedMessageCompanion[_] = null
    (__number: @_root_.scala.unchecked) match {
      case 3 => __out = lbir.QTensor
      case 4 => __out = lbir.QTensor
      case 5 => __out = lbir.Quantizer
    }
    __out
  }
  lazy val nestedMessagesCompanions: Seq[_root_.scalapb.GeneratedMessageCompanion[_ <: _root_.scalapb.GeneratedMessage]] = Seq.empty
  def enumCompanionForFieldNumber(__fieldNumber: _root_.scala.Int): _root_.scalapb.GeneratedEnumCompanion[_] = {
    (__fieldNumber: @_root_.scala.unchecked) match {
      case 1 => lbir.Layer.LayerType
    }
  }
  lazy val defaultInstance = lbir.Layer(
    layerType = lbir.Layer.LayerType.DENSE,
    useBias = false,
    weights = _root_.scala.None,
    biases = _root_.scala.None,
    inputQuantizer = _root_.scala.None,
    width = 0,
    height = 0,
    channels = 0
  )
  sealed abstract class LayerType(val value: _root_.scala.Int) extends _root_.scalapb.GeneratedEnum {
    type EnumType = LayerType
    def isDense: _root_.scala.Boolean = false
    def isConv2D: _root_.scala.Boolean = false
    def companion: _root_.scalapb.GeneratedEnumCompanion[LayerType] = lbir.Layer.LayerType
    final def asRecognized: _root_.scala.Option[lbir.Layer.LayerType.Recognized] = if (isUnrecognized) _root_.scala.None else _root_.scala.Some(this.asInstanceOf[lbir.Layer.LayerType.Recognized])
  }
  
  object LayerType extends _root_.scalapb.GeneratedEnumCompanion[LayerType] {
    sealed trait Recognized extends LayerType
    implicit def enumCompanion: _root_.scalapb.GeneratedEnumCompanion[LayerType] = this
    @SerialVersionUID(0L)
    case object DENSE extends LayerType(0) with LayerType.Recognized {
      val index = 0
      val name = "DENSE"
      override def isDense: _root_.scala.Boolean = true
    }
    
    @SerialVersionUID(0L)
    case object CONV2D extends LayerType(1) with LayerType.Recognized {
      val index = 1
      val name = "CONV2D"
      override def isConv2D: _root_.scala.Boolean = true
    }
    
    @SerialVersionUID(0L)
    final case class Unrecognized(unrecognizedValue: _root_.scala.Int) extends LayerType(unrecognizedValue) with _root_.scalapb.UnrecognizedEnum
    
    lazy val values = scala.collection.immutable.Seq(DENSE, CONV2D)
    def fromValue(__value: _root_.scala.Int): LayerType = __value match {
      case 0 => DENSE
      case 1 => CONV2D
      case __other => Unrecognized(__other)
    }
    def javaDescriptor: _root_.com.google.protobuf.Descriptors.EnumDescriptor = lbir.Layer.javaDescriptor.getEnumTypes().get(0)
    def scalaDescriptor: _root_.scalapb.descriptors.EnumDescriptor = lbir.Layer.scalaDescriptor.enums(0)
  }
  implicit class LayerLens[UpperPB](_l: _root_.scalapb.lenses.Lens[UpperPB, lbir.Layer]) extends _root_.scalapb.lenses.ObjectLens[UpperPB, lbir.Layer](_l) {
    def layerType: _root_.scalapb.lenses.Lens[UpperPB, lbir.Layer.LayerType] = field(_.layerType)((c_, f_) => c_.copy(layerType = f_))
    def useBias: _root_.scalapb.lenses.Lens[UpperPB, _root_.scala.Boolean] = field(_.useBias)((c_, f_) => c_.copy(useBias = f_))
    def weights: _root_.scalapb.lenses.Lens[UpperPB, lbir.QTensor] = field(_.getWeights)((c_, f_) => c_.copy(weights = Option(f_)))
    def optionalWeights: _root_.scalapb.lenses.Lens[UpperPB, _root_.scala.Option[lbir.QTensor]] = field(_.weights)((c_, f_) => c_.copy(weights = f_))
    def biases: _root_.scalapb.lenses.Lens[UpperPB, lbir.QTensor] = field(_.getBiases)((c_, f_) => c_.copy(biases = Option(f_)))
    def optionalBiases: _root_.scalapb.lenses.Lens[UpperPB, _root_.scala.Option[lbir.QTensor]] = field(_.biases)((c_, f_) => c_.copy(biases = f_))
    def inputQuantizer: _root_.scalapb.lenses.Lens[UpperPB, lbir.Quantizer] = field(_.getInputQuantizer)((c_, f_) => c_.copy(inputQuantizer = Option(f_)))
    def optionalInputQuantizer: _root_.scalapb.lenses.Lens[UpperPB, _root_.scala.Option[lbir.Quantizer]] = field(_.inputQuantizer)((c_, f_) => c_.copy(inputQuantizer = f_))
    def width: _root_.scalapb.lenses.Lens[UpperPB, _root_.scala.Int] = field(_.width)((c_, f_) => c_.copy(width = f_))
    def height: _root_.scalapb.lenses.Lens[UpperPB, _root_.scala.Int] = field(_.height)((c_, f_) => c_.copy(height = f_))
    def channels: _root_.scalapb.lenses.Lens[UpperPB, _root_.scala.Int] = field(_.channels)((c_, f_) => c_.copy(channels = f_))
  }
  final val LAYERTYPE_FIELD_NUMBER = 1
  final val USE_BIAS_FIELD_NUMBER = 2
  final val WEIGHTS_FIELD_NUMBER = 3
  final val BIASES_FIELD_NUMBER = 4
  final val INPUTQUANTIZER_FIELD_NUMBER = 5
  final val WIDTH_FIELD_NUMBER = 6
  final val HEIGHT_FIELD_NUMBER = 7
  final val CHANNELS_FIELD_NUMBER = 8
  def of(
    layerType: lbir.Layer.LayerType,
    useBias: _root_.scala.Boolean,
    weights: _root_.scala.Option[lbir.QTensor],
    biases: _root_.scala.Option[lbir.QTensor],
    inputQuantizer: _root_.scala.Option[lbir.Quantizer],
    width: _root_.scala.Int,
    height: _root_.scala.Int,
    channels: _root_.scala.Int
  ): _root_.lbir.Layer = _root_.lbir.Layer(
    layerType,
    useBias,
    weights,
    biases,
    inputQuantizer,
    width,
    height,
    channels
  )
  // @@protoc_insertion_point(GeneratedMessageCompanion[chisel4ml.Layer])
}
