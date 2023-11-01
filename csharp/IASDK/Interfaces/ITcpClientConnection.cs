/*
Copyright 2016 Navtech Radar Limited
This file is part of iasdk which is released under The MIT License (MIT).
See file LICENSE.txt in project root or go to https://opensource.org/licenses/MIT 
for full license details.
*/

using Navtech.IASDK.Events;
using System;
using System.Net.Sockets;

namespace Navtech.IASDK.Interfaces
{
    /// <summary>
    /// Represents a client connection within the TCP Server
    /// </summary>
    /// <typeparam name="T">Type of message to be handled</typeparam>
    public interface ITcpClientConnection<T> : IDisposable
    {
        /// <summary>
        /// Informs subscribers that a message of type T has been received
        /// </summary>
        event EventHandler<GenericEventArgs<T>> OnData;
        /// <summary>
        /// The IP address of the connected client
        /// </summary>
        string ClientIpAddress { get; }
        /// <summary>
        /// A property that exposes the TcpClient
        /// </summary>
        TcpClient TcpClient { get; set; }
        /// <summary>
        /// Disconnects the server from this specific client
        /// </summary>
        void Disconnect();
        /// <summary>
        /// Sends a compatible raw data message to the client
        /// <see cref="IRawData"/>
        /// </summary>
        /// <param name="rawData">Compatiable raw data message</param>
        void SendData(IRawData rawData);
    }
}
