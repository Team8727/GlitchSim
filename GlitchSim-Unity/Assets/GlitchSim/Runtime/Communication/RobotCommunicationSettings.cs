using UnityEngine;

namespace GlitchSim.Runtime.Communication
{
    /// <summary>
    /// Settings for a <see cref="RobotCommunication"/> instance.
    /// </summary>
    [CreateAssetMenu(fileName = "RobotCommunicationSettings", menuName = "GlitchSim/RobotCommunicationSettings")]
    public class RobotCommunicationSettings : ScriptableObject
    {
        [SerializeField] private string _hostName = "ws://localhost:8080";
        public string HostName => _hostName;
    }
}