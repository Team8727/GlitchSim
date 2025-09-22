#if _MSC_VER // this is defined when compiling with Visual Studio
#define EXPORT_API __declspec(dllexport) // Visual Studio needs annotating exported functions with this
#else
#define EXPORT_API // XCode does not need annotating exported functions, so define is empty
#endif

#include "ntcore_cpp.h"

extern "C"
{
    EXPORT_API NT_Inst GetDefaultInstance() {
        return nt::GetDefaultInstance();
    }

    EXPORT_API void StartClient(const NT_Inst instance, const char* clientName, const char* host, const unsigned int port) {
        nt::StartClient4(instance, clientName);
        nt::SetServer(instance, host, port);
    }

    EXPORT_API NT_Subscriber Subscribe(const NT_Inst instance, const char* topicName, const NT_Type type, const char* typeStr) {
        return nt::Subscribe(nt::GetTopic(instance, topicName), type, typeStr);
    }

    EXPORT_API bool GetBoolean(const NT_Subscriber subscriber) {
        return nt::GetBoolean(subscriber, false);
    }

    EXPORT_API float GetFloat(const NT_Subscriber subscriber) {
        return nt::GetFloat(subscriber, 0.0f);
    }

    EXPORT_API double GetDouble(const NT_Subscriber subscriber) {
        return nt::GetDouble(subscriber, 0.0);
    }
}