/*
Copyright 2016 Navtech Radar Limited
This file is part of iasdk which is released under The MIT License (MIT).
See file LICENSE.txt in project root or go to https://opensource.org/licenses/MIT 
for full license details.
*/

namespace Navtech.IASDK.Networking
{
    /// <summary>
    /// Network utility class
    /// </summary>
    public static class Utility
    {
        /// <summary>
        /// Swaps bytes in an Int32
        /// </summary>
        /// <param name="value">Int32 to manipulate</param>
        /// <returns>Corrected Int32 value</returns>
        public static int SwapInt32(int value)
        {
            return ((SwapUInt16((ushort)((value & 0x000ffff))) << 0x10)) | (SwapUInt16((ushort)((value >> 0x10) & 0xffff)));
        }

        /// <summary>
        /// Swaps bytes in an UInt32
        /// </summary>
        /// <param name="value">UInt32 to manipulate</param>
        /// <returns>Corrected UInt32 value</returns>
        public static uint SwapUInt32(uint value)
        {
            return (uint)((SwapUInt16((ushort)((value & 0x000ffff))) << 0x10)) | (SwapUInt16((ushort)((value >> 0x10) & 0xffff)));
        }

        /// <summary>
        /// Swaps bytes in an UInt16
        /// </summary>
        /// <param name="value">UInt16 to manipulate</param>
        /// <returns>Corrected Uint16 value</returns>
        public static ushort SwapUInt16(ushort value)
        {
            return (ushort)(((value & 0xff) << 8) | ((value >> 8) & 0xff));
        }
    }
}
