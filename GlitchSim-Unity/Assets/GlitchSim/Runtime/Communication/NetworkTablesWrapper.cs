using System.Runtime.InteropServices;

namespace GlitchSim.Runtime.Communication
{
    public static class NetworkTablesWrapper
    {
        [DllImport("ntcore_unity_wrapper", CallingConvention = CallingConvention.Cdecl)]
        public static extern float test();
    }
}