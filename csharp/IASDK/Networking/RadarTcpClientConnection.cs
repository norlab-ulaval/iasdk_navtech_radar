/*
Copyright 2016 Navtech Radar Limited
This file is part of iasdk which is released under The MIT License (MIT).
See file LICENSE.txt in project root or go to https://opensource.org/licenses/MIT 
for full license details.
*/

namespace Navtech.IASDK.Networking
{
    /// <summary>
    /// Represents a client radar connection within the TCP Server
    /// This is specialised to handle radar specific connections
    /// with clients
    /// </summary>
    public class RadarTcpClientConnection : TcpClientConnection<TcpNetworkDataMessage>
    {
        /// <summary>
        /// Overridable method that extracts Colossus network data messages
        /// from the TCP stream
        /// </summary>
        /// <returns>Returns a <see cref="TcpNetworkDataMessage"/></returns>
        protected override TcpNetworkDataMessage ExtractFromStream()
        {
            var headerData = new byte[TcpNetworkDataMessage.HeaderLength];
            headerData.Initialize();
            var headerBytesRead = 0;
            while (headerBytesRead != TcpNetworkDataMessage.HeaderLength)
            {
                headerBytesRead += ClientStream.Read(headerData, headerBytesRead, TcpNetworkDataMessage.HeaderLength - headerBytesRead);
            }
            var message = new TcpNetworkDataMessage(headerData);
            if (!message.Valid) return null;

            var bytesRead = 0;
            while (bytesRead < message.PayloadSize)
            {
                bytesRead += ClientStream.Read(message.Payload, bytesRead, (int)message.PayloadSize - bytesRead);
            }

            return message;
        }
    }
}
