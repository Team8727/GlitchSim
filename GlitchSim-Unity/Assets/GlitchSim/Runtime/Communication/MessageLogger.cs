using System;
using GlitchSim.Runtime.Core;
using UnityEngine;
using UnityEngine.Serialization;

namespace GlitchSim.Runtime.Communication
{
    /// <summary>
    /// Finds the communication service once it's registered and logs the specified incoming messages
    /// </summary>
    public class MessageLogger : MonoBehaviour
    {
        [Serializable]
        public class Topic
        {
            public string name;
            public ICommunication.DataType type;
        }

        [SerializeField] private Topic[] topics;
        
        private ICommunication _communication;
        
        private void OnEnable()
        {
            StartCoroutine(ServiceLocator.TryGet<ICommunication>(OnCommunicationServiceLocated, 1.0f));
        }

        private void OnCommunicationServiceLocated(ICommunication communication)
        {
            _communication = communication;
            
            if (_communication != null)
            {
                _communication.OnMessageReceived += OnMessageReceived;

                foreach (var topic in topics)
                {
                    _communication.Subscribe(topic.name, topic.type);
                }
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