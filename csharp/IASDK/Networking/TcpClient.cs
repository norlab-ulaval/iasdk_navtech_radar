/*
Copyright 2016 Navtech Radar Limited
This file is part of iasdk which is released under The MIT License (MIT).
See file LICENSE.txt in project root or go to https://opensource.org/licenses/MIT 
for full license details.
*/

using Navtech.IASDK.Enums;
using Navtech.IASDK.Events;
using Navtech.IASDK.Interfaces;
using System;
using System.Diagnostics;
using System.Threading;

namespace Navtech.IASDK.Networking
{
    /// <summary>
    /// An generic TCP Client that can be used to handle specific
    /// messaging types through the use of the generic parameters
    /// This class utilises the <see cref="ASyncTcpClient"/> class
    /// <seealso cref="RadarTcpClient"/>
    /// </summary>
    /// <typeparam name="T"></typeparam>
    /// <typeparam name="TU"></typeparam>
    public class TcpClient<T, TU> where T : ITcpClientConnection<TU>, new() where TU : new()
    {
        /// <summary>
        /// Informs subscribers when a data message has arrived of type TU
        /// </summary>
        public event EventHandler<GenericEventArgs<TU>> OnData;
        /// <summary>
        /// Informs subscribers when the client connection status has changed
        /// </summary>
        public event EventHandler<ConnectionStateEventArgs> OnConnectionChanged;

        private string _serverAddress;
        private ushort _port;
        private ASyncTcpClient _tcpClient;
        private ITcpClientConnection<TU> _client;

        /// <summary>
        /// Default constructor
        /// </summary>
        public TcpClient()
        {
            _tcpClient = new ASyncTcpClient();
        }

        /// <summary>
        /// Connects to a radar using the specified IP address, port and connection timeout
        /// </summary>
        /// <param name="ipAddress">IP address of radar</param>
        /// <param name="port">Optional port - leave blank to use default</param>
        /// <param name="timeout">Optional connection timeout - leave blank to use default</param>
        public void Connect(string ipAddress, ushort port = 6317, int timeout = 2000)
        {
            _serverAddress = ipAddress;
            _port = port;

            if (_tcpClient.Connected) return;

            try
            {

                OnConnectionChanged?.Invoke(this, new ConnectionStateEventArgs(ConnectionState.Connecting));
                _tcpClient.Connect(_serverAddress, _port, timeout);
                if (_tcpClient.Connected)
                {
                    _client = new T { TcpClient = _tcpClient };
                    _client.OnData += OnDataHandler;

                    OnConnectionChanged?.Invoke(this, new ConnectionStateEventArgs(ConnectionState.Connected));
                    return;
                }

            }
            catch (Exception ex)
            {
                Debug.WriteLine($"Failed to connect to server: [{ex.Message}]");
            }

            Disconnect();
        }

        private void OnDataHandler(object sender, GenericEventArgs<TU> e)
        {
            OnData?.Invoke(sender, e);
        }

        /// <summary>
        /// Sends a compatible raw data message to the radar
        /// <see cref="IRawData"/>
        /// </summary>
        /// <param name="data">Compatiable raw data message</param>
        public void SendData(IRawData data)
        {
            try
            {
                _client.SendData(data);
                if (!_tcpClient.Connected) Disconnect();
            }
            catch (Exception ex)
            {
                Debug.WriteLine($"Failed to send data to server: [{_serverAddress}]. Server connection will be closed: [{ex.Message}]");
                Disconnect();
            }
        }

        /// <summary>
        /// Disconnects the client from the radar
        /// </summary>
        public void Disconnect()
        {
            if (_client != null)
            {
                _client.OnData -= OnDataHandler;
                _client.Disconnect();
            }
            _tcpClient.Close();
            _tcpClient = new ASyncTcpClient();
            ThreadPool.QueueUserWorkItem(state => OnConnectionChanged?.Invoke(this, new ConnectionStateEventArgs(ConnectionState.Disconnected)));
        }
    }
}
