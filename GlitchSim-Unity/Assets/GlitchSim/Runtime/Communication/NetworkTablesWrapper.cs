using System;
using System.Runtime.InteropServices;

namespace GlitchSim.Runtime.Communication
{
    public class NetworkTablesWrapper
    {
        public enum TopicType {
            Unassigned = 0,
            Boolean = 0x01,
            Double = 0x02,
            String = 0x04,
            Raw = 0x08,
            BooleanArray = 0x10,
            DoubleArray = 0x20,
            StringArray = 0x40,
            RPC = 0x80,
            Integer = 0x100,
            Float = 0x200,
            IntegerArray = 0x400,
            FloatArray = 0x800
        }

        private static string TopicTypeToString(TopicType type)
        {
            return type switch
            {
                TopicType.Unassigned => "Unassigned",
                TopicType.Boolean => "Boolean",
                TopicType.Double => "Double",
                TopicType.String => "String",
                TopicType.Raw => "Raw",
                TopicType.BooleanArray => "BooleanArray",
                TopicType.DoubleArray => "DoubleArray",
                TopicType.StringArray => "StringArray",
                TopicType.RPC => "RPC",
                TopicType.Integer => "Integer",
                TopicType.Float => "Float",
                TopicType.IntegerArray => "IntegerArray",
                TopicType.FloatArray => "FloatArray",
                _ => throw new ArgumentOutOfRangeException(nameof(type), type, null)
            };
        }

        public class Subscriber
        {
            public class Message<T> : IMessage
            {
                public readonly T Value;
                
                public Message(T value)
                {
                    Value = value;
                }
                
                public override string ToString()
                {
                    return Value.ToString();
                }
            }

            public readonly string TopicName;
            public readonly TopicType Type;
            
            private readonly uint _handle;

            public Subscriber(uint instance, string topicName, TopicType type)
            {
                _handle = Subscribe(instance, topicName, type, TopicTypeToString(type));
                TopicName = topicName;
                Type = type;
            }

            public IMessage GetMessage()
            {
                switch (Type)
                {
                    case TopicType.Boolean:
                        return new Message<bool>(GetBoolean());
                    case TopicType.Float:
                        return new Message<float>(GetFloat());
                    case TopicType.Double:
                        return new Message<double>(GetDouble());
                    case TopicType.String:
                    case TopicType.Unassigned:
                    case TopicType.Raw:
                    case TopicType.BooleanArray:
                    case TopicType.DoubleArray:
                    case TopicType.StringArray:
                    case TopicType.RPC:
                    case TopicType.IntegerArray:
                    case TopicType.FloatArray:
                        return new Message<string>($"Unsupported type: {Type}");
                    default:
                        throw new ArgumentOutOfRangeException();
                }
            }

            private bool GetBoolean()
            {
                return NetworkTablesWrapper.GetBoolean(_handle);
            }

            private float GetFloat()
            {
                return NetworkTablesWrapper.GetFloat(_handle);
            }

            private double GetDouble()
            {
                return NetworkTablesWrapper.GetDouble(_handle);
            }
        }
        
        private uint _instance;

        public void Connect(string clientName, string host, uint port)
        {
            // TODO: verify connection
            _instance = GetDefaultInstance();
            StartClient(_instance, clientName, host, port);
        }

        public Subscriber Subscribe(string topicName, TopicType type)
        {
            return new Subscriber(_instance, topicName, type);
        }
        
        [DllImport("ntcore_unity_wrapper", CallingConvention = CallingConvention.Cdecl)]
        private static extern uint GetDefaultInstance();
        
        [DllImport("ntcore_unity_wrapper", CallingConvention = CallingConvention.Cdecl)]
        private static extern void StartClient(uint instance, string clientName, string host, uint port);
        
        [DllImport("ntcore_unity_wrapper", CallingConvention = CallingConvention.Cdecl)]
        private static extern uint Subscribe(uint instance, string topicName, TopicType type, string typeString);
        
        [DllImport("ntcore_unity_wrapper", CallingConvention = CallingConvention.Cdecl)]
        private static extern bool GetBoolean(uint subscriber);

        [DllImport("ntcore_unity_wrapper", CallingConvention = CallingConvention.Cdecl)]
        private static extern float GetFloat(uint subscriber);
        
        [DllImport("ntcore_unity_wrapper", CallingConvention = CallingConvention.Cdecl)]
        private static extern double GetDouble(uint subscriber);
    }
}