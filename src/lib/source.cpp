#include "source.hpp"

#include <cstring>
#include <unistd.h>

using namespace Csdr;

template <typename T>
void Source<T>::setWriter(Writer<T>* writer) {
    this->writer = writer;
}

template <typename T>
Writer<T>* Source<T>::getWriter() {
    return writer;
}

template<typename T>
bool Source<T>::hasWriter() {
    return writer != nullptr;
}

template<typename T>
TcpSource<T>::~TcpSource() {
    ::close(sock);
}

template <typename T>
TcpSource<T>::TcpSource(in_addr_t ip, unsigned short port) {
    sockaddr_in remote{};
    std::memset(&remote, 0, sizeof(remote));

    remote.sin_family = AF_INET;
    remote.sin_port = htons(port);
    remote.sin_addr.s_addr = ip;

    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        throw NetworkException("unable to create socket");
    }

    if (connect(sock, (struct sockaddr*)&remote, sizeof(remote)) < 0) {
        throw NetworkException("connection failed");
    }

}

template <typename T>
void TcpSource<T>::setWriter(Writer<T> *writer) {
    Source<T>::setWriter(writer);
    if (thread == nullptr) {
        thread = new std::thread( [this] () { loop(); });
    }
}

template <typename T>
void TcpSource<T>::loop() {
    int read_bytes;
    int available;
    int offset = 0;

    while (run) {
        available = std::min(this->writer->writeable(), (size_t) 1024);
        read_bytes = recv(sock, this->writer->getWritePointer() + offset, available * sizeof(T) - offset, 0);
        if (read_bytes <= 0) {
            run = false;
        } else {
            offset = (offset + read_bytes) % sizeof(T);
            this->writer->advance((offset + read_bytes) / sizeof(T));
        }
    }
}

template <typename T>
void TcpSource<T>::stop() {
    run = false;
    if (thread != nullptr) {
        thread->join();
        delete(thread);
    }
}

namespace Csdr {
    template class Source<float>;
    template class Source<short>;
    template class Source<complex<float>>;
    template class Source<unsigned char>;

    template class TcpSource<complex<float>>;
}