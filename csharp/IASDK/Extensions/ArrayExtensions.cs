/*
Copyright 2016 Navtech Radar Limited
This file is part of iasdk which is released under The MIT License (MIT).
See file LICENSE.txt in project root or go to https://opensource.org/licenses/MIT 
for full license details.
*/

using System;

namespace Navtech.IASDK.Extensions
{
    /// <summary>
    /// Extension methods for arrays
    /// </summary>
    public static class ArrayExtensions
    {
        /// <summary>
        /// Takes a slice of an existing array and returns a new array
        /// </summary>
        /// <typeparam name="T"></typeparam>
        /// <param name="source"></param>
        /// <param name="start"></param>
        /// <param name="end"></param>
        /// <returns></returns>
        public static T[] Slice<T>(this T[] source, int start, int end)
        {
            // Handles negative ends
            if (end < 0)
                end = source.Length - start - end - 1;

            var len = end - start;

            // Return new array
            var res = new T[len];
            Buffer.BlockCopy(source, start, res, 0, end - start);
            return res;
        }
    }
}
