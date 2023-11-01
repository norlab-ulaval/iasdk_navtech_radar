using System;
using System.Collections.Generic;

namespace Navtech.IASDK.Networking
{
    /// <summary>
    /// Represents an extracted peak - includes range and power
    /// </summary>
    public class NavigationReturn
    {
        /// <summary>
        /// Range to peak
        /// </summary>
        public double Range { get; set; }

        /// <summary>
        /// Signal strength of the peak
        /// </summary>
        public ushort Power { get; set; }
    }

    /// <summary>
    /// A navigation method that includes the azimuth and time stamp
    /// and a list of extracted peeks on the reported azimuth
    /// This message is generated for each reported azimuth
    /// </summary>
    public class NavigationData
    {
        /// <summary>
        /// Navigation data message that includes the azimuth and timestamp
        /// </summary>
        public TcpNavigationDataMessage Message { get; private set; }

        /// <summary>
        /// List of extracted peaks <see cref="NavigationReturns"/>
        /// </summary>
        public List<NavigationReturn> NavigationReturns { get; private set; }

        /// <summary>
        /// Default constructor
        /// </summary>
        /// <param name="message">Navigation data message</param>
        public NavigationData(TcpNavigationDataMessage message)
        {
            Message = message;
            NavigationReturns = new List<NavigationReturn>();
        }
    }

    /// <summary>
    /// Provides EventArgs to pass the Navigation Data message to subscribers
    /// <see cref="NavigationData"/>
    /// </summary>
    public class NavigationDataEventArgs : EventArgs
    {
        /// <summary>
        /// Navigation data message
        /// </summary>
        public NavigationData Data { get; set; }

        /// <summary>
        /// Default constructor
        /// </summary>
        /// <param name="data">The Navigation Data message to pass to subscribers</param>
        public NavigationDataEventArgs(NavigationData data)
        {
            Data = data;
        }
    }
}
