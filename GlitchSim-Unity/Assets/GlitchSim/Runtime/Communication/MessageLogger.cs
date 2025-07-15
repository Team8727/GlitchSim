using GlitchSim.Runtime.Core;
using UnityEngine;

namespace GlitchSim.Runtime.Communication
{
    /// <summary>
    /// Finds the communication service once it's registered and logs any incoming messages
    /// </summary>
    public class MessageLogger : MonoBehaviour
    {
        private ICommunication _communication;
        
        private void OnEnable()
        {
            StartCoroutine(ServiceLocator.TryGet<ICommunication>(OnCommunicationServiceLocated, 1.0f));
        }

        private void OnCommunicationServiceLocated(ICommunication obj)
        {
            if (_communication != null)
            {
                _communication.OnMessageReceived -= OnMessageReceived;
            }
        }

        private void OnDisable()
        {
            if (_communication != null)
            {
                _communication.OnMessageReceived -= OnMessageReceived;
            }
        }

        private static void OnMessageReceived(IMessage message)
        {
            Debug.Log(message);
        }
    }
}