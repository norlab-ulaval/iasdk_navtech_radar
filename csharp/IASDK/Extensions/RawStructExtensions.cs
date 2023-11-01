/*
Copyright 2016 Navtech Radar Limited
This file is part of iasdk which is released under The MIT License (MIT).
See file LICENSE.txt in project root or go to https://opensource.org/licenses/MIT 
for full license details.
*/

using System.Runtime.InteropServices;

namespace Navtech.IASDK.Extensions
{
    /// <summary>
    /// Extensions for byte[]
    /// </summary>
    public static class RawStructExtensions
    {
        /// <summary>
        /// Marshalls a byte array into the supplied type
        /// </summary>
        /// <typeparam name="T">The type to construct from the bytes</typeparam>
        /// <param name="data">The byte data to use to create the type</param>
        /// <returns>An instance of type T</returns>
        public static T MarshalToObject<T>(this byte[] data) where T : struct
        {
            var rawsize = Marshal.SizeOf(typeof(T));
            if (rawsize > data.Length)
                return new T();

            var buffer = Marshal.AllocHGlobal(rawsize);
            Marshal.Copy(data, 0, buffer, rawsize);
            var retobj = Marshal.PtrToStructure(buffer, typeof(T));
            Marshal.FreeHGlobal(buffer);
            return (T)retobj;
        }
    }
}
