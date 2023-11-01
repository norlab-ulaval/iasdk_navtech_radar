/*
Copyright 2016 Navtech Radar Limited
This file is part of iasdk which is released under The MIT License (MIT).
See file LICENSE.txt in project root or go to https://opensource.org/licenses/MIT 
for full license details.
*/

using Navtech.IASDK.Enums;
using Navtech.IASDK.Extensions;
using Navtech.IASDK.Interfaces;
using System;
using System.Linq;

namespace Navtech.IASDK.Networking
{
    /// <summary>
    /// Represents core Colossus network data message
    /// </summary>
    public class TcpNetworkDataMessage : IRawData
    {
        private const int SignatureLength = 16;
        private const int VersionLength = 1;
        private const int MessageTypeLength = 1;
        private const int PayloadLength = 4;
        private const int ProtocolVersion = 1;

        private static readonly byte[] Signature = { 0x00, 0x01, 0x03, 0x03, 0x07, 0x07, 0x0f, 0x0f, 0x1f, 0x1f, 0x3f, 0x3f, 0x7f, 0x7f, 0xfe, 0xfe };
        private readonly byte[] _signature = new byte[SignatureLength];

        /// <summary>
        /// Default constructor
        /// </summary>
        public TcpNetworkDataMessage() { }

        /// <summary>
        /// Constructor to create a data message
        /// based on the supplied message type
        /// </summary>
        /// <param name="messageType">Required message type</param>
        public TcpNetworkDataMessage(TcpNetworkDataMessageType messageType)
        {
            Payload = null;
            MessageType = messageType;
            Version = ProtocolVersion;
        }

        /// <summary>
        /// Constructor to create a data message
        /// based on the supplied message type and byte payload
        /// </summary>
        /// <param name="messageType">Required message type</param>
        /// <param name="payload">Message payload in bytes</param>
        public TcpNetworkDataMessage(TcpNetworkDataMessageType messageType, byte[] payload)
        {
            Payload = payload;
            MessageType = messageType;
            PayloadSize = Payload.Length;
            Version = ProtocolVersion;
        }

        /// <summary>
        /// Constructor to create a complete data message
        /// based on the supplied byte payload
        /// </summary>
        /// <param name="data">The full message in bytes</param>
        public TcpNetworkDataMessage(byte[] data)
        {
            Buffer.BlockCopy(data, 0, _signature, 0, SignatureLength);
            var index = SignatureLength;
            Version = data.Slice(index, index + VersionLength)[0];
            index += VersionLength;
            MessageType = (TcpNetworkDataMessageType)Enum.ToObject(typeof(TcpNetworkDataMessageType), data.Slice(index, index + MessageTypeLength)[0]);
            index += MessageTypeLength;
            PayloadSize = Utility.SwapInt32(BitConverter.ToInt32(data.Slice(index, index + PayloadLength), 0));
            index += PayloadLength;
            Payload = new byte[PayloadSize];
            if (data.Length == HeaderLength + PayloadSize)
                Buffer.BlockCopy(data.Slice(index, index + PayloadSize), 0, Payload, 0, PayloadSize);
        }

        /// <summary>
        /// Static value that provides the full header length in bytes
        /// </summary>
        public static byte HeaderLength => SignatureLength + VersionLength + MessageTypeLength + PayloadLength;

        /// <summary>
        /// Property that indicates if the message, having been decoded, is valid
        /// </summary>
        public bool Valid => _signature.SequenceEqual(Signature);

        /// <summary>
        /// Property that gives the message version
        /// </summary>
        public byte Version { get; private set; }

        /// <summary>
        /// The type of the message <see cref="TcpNetworkDataMessageType"/>
        /// </summary>
        public TcpNetworkDataMessageType MessageType { get; private set; }

        /// <summary>
        /// Size of just the message payload
        /// </summary>
        public int PayloadSize { get; private set; }

        /// <summary>
        /// Total message size based on header size + payload size
        /// </summary>
        public int TotalLength => HeaderLength + PayloadSize;

        /// <summary>
        /// Message payload in bytes
        /// </summary>
        public byte[] Payload { get; set; }

        private byte[] HeaderBytes()
        {
            var buffer = new byte[HeaderLength];
            var payloadLength = BitConverter.GetBytes(Utility.SwapInt32(PayloadSize));
            Buffer.BlockCopy(Signature, 0, buffer, 0, SignatureLength);
            buffer[SignatureLength] = Version;
            buffer[SignatureLength + VersionLength] = (byte)MessageType;
            Buffer.BlockCopy(payloadLength, 0, buffer, SignatureLength + VersionLength + MessageTypeLength, PayloadLength);
            return buffer;
        }

        /// <summary>
        /// Property that encodes all properties and includes the payload
        /// to provide a single byte array for the complete message
        /// </summary>
        public byte[] Data
        {
            get
            {
                var buffer = new byte[TotalLength];
                Buffer.BlockCopy(HeaderBytes(), 0, buffer, 0, HeaderLength);
                if (Payload != null)
                    Buffer.BlockCopy(Payload, 0, buffer, HeaderLength, TotalLength - HeaderLength);
                return buffer;
            }
        }
    }
}
