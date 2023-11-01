/*
Copyright 2016 Navtech Radar Limited
This file is part of iasdk which is released under The MIT License (MIT).
See file LICENSE.txt in project root or go to https://opensource.org/licenses/MIT 
for full license details.
*/

using System;

namespace Navtech.IASDK.Events
{
    /// <summary>
    /// Fully generic EventArgs
    /// </summary>
    /// <typeparam name="T">Type of EventArgs payload</typeparam>
    public class GenericEventArgs<T> : EventArgs
    {
        /// <summary>
        /// Payload of generic EventArgs
        /// </summary>
        public T Payload { get; set; }
    }
}
