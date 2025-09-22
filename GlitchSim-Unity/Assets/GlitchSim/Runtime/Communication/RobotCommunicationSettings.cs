using UnityEngine;
using UnityEngine.Serialization;

namespace GlitchSim.Runtime.Communication
{
    /// <summary>
    /// Settings for a <see cref="RobotCommunication"/> instance.
    /// </summary>
    [CreateAssetMenu(fileName = "RobotCommunicationSettings", menuName = "GlitchSim/RobotCommunicationSettings")]
    public class RobotCommunicationSettings : ScriptableObject
    {
        [SerializeField] private string hostName = "ws://localhost";
        
        [SerializeField] private uint port = 8080;
        
        public string HostName => hostName;
        
        public uint Port => port;
    }
}