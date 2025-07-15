using System;
using GlitchSim.Runtime.Communication;
using UnityEngine;

namespace GlitchSim.Runtime.Core
{
    /// <summary>
    /// The entry point that handles loading the rest of the simulation environment.
    /// </summary>
    public class GlitchSimMain : MonoBehaviour
    {
        [SerializeField] private RobotCommunicationSettings robotCommunicationSettings;
        
        private RobotCommunication _robotCommunication;

        private void Awake()
        {
            _robotCommunication = new RobotCommunication(robotCommunicationSettings);
            ServiceLocator.Register<ICommunication>(_robotCommunication);
        }

        private void OnDestroy()
        {
            ServiceLocator.Unregister<ICommunication>();
        }

        private void Update()
        {
            _robotCommunication.DispatchMessageQueue();
        }
    }
}
