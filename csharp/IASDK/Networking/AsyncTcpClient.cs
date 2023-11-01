/*
Copyright 2016 Navtech Radar Limited
This file is part of iasdk which is released under The MIT License (MIT).
See file LICENSE.txt in project root or go to https://opensource.org/licenses/MIT 
for full license details.
*/

using System;
using System.Net;
using System.Net.Sockets;
using System.Threading;

namespace Navtech.IASDK.Networking
{
    /// <summary>
    /// Provides an Async wrapper around the .NET TcpClient
    /// </summary>
    public class ASyncTcpClient : TcpClient
    {
        private readonly ManualResetEvent _timeoutGate = new ManualResetEvent(false);
        private volatile bool _isConnectionSuccesful;
        private Exception _socketException;

        /// <summary>
        /// Connects asynchronously the client to the specified port on the specified host.
        /// </summary>
        /// <param name="hostname">The DNS name of the remote host to which you intend to connect.</param>
        /// <param name="port">The port number of the remote host to which you intend to connect.</param>
        /// <param name="timeOut">Time to wait whilst trying to connect in ms.</param>
        /// <exception cref="ArgumentNullException">The host name parameter is a null reference (Nothing in Visual Basic).</exception>
        /// <exception cref="ArgumentOutOfRangeException">The port is not between MinPort and MaxPort.</exception>
        /// <exception cref="SocketException">An error occurred when accessing the socket.</exception>
        /// <exception cref="ObjectDisposedException">TcpClient is closed.</exception>
        public void Connect(string hostname, int port, int timeOut)
        {
            IPAddress ipAddress;
            if (IPAddress.TryParse(hostname, out ipAddress))
            {
                Connect(ipAddress, port, timeOut);
            }
            else
                ConnectHost(hostname, port, timeOut);
        }

        private void Connect(IPAddress address, int port, int timeOut)
        {
            var ep = new IPEndPoint(address, port);
            Connect(ep, timeOut);
        }

        private void Connect(IPEndPoint remoteEp, int timeOut)
        {
            BeginConnect(remoteEp.Address, remoteEp.Port, CallBackMethod, this);

            if (_timeoutGate.WaitOne(timeOut, false))
            {
                _timeoutGate.Close();
                if (_isConnectionSuccesful)
                    return;

                throw _socketException ?? new NullReferenceException();
            }

            Close();
            _timeoutGate.Close();
            throw new TimeoutException("Connection timed out");
        }

        private void ConnectHost(string hostname, int port, int timeOut)
        {
            BeginConnect(hostname, port, CallBackMethod, this);

            if (_timeoutGate.WaitOne(timeOut, false))
            {
                _timeoutGate.Close();

                if (_isConnectionSuccesful)
                    return;

                throw _socketException ?? new NullReferenceException();
            }

            Close();
            _timeoutGate.Close();
            throw new TimeoutException("Connection timed out");
        }

        private void CallBackMethod(IAsyncResult asyncResult)
        {
            try
            {
                _isConnectionSuccesful = false;
                if (Client == null) return;
                EndConnect(asyncResult);
                _isConnectionSuccesful = true;
                _timeoutGate.Set();
            }
            catch (Exception ex)
            {
                _isConnectionSuccesful = false;
                _socketException = ex;
            }
        }
    }
}
