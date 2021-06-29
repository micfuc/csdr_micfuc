#include "commands.hpp"
#include "agc.hpp"
#include "fmdemod.hpp"
#include "amdemod.hpp"

#include <iostream>
#include <errno.h>
#include <cstring>

using namespace Csdr;

template <typename T, typename U>
void Command::runModule(Module<T, U>* module) {
    Ringbuffer<T>* buffer = new Ringbuffer<T>(10240);
    module->setReader(new RingbufferReader<T>(buffer));
    module->setWriter(new StdoutWriter<U>());

    fd_set read_fds;
    struct timeval tv;
    int rc;
    int nfds = fileno(stdin) + 1;
    size_t read;
    size_t read_over = 0;

    bool run = true;
    while (run) {
        FD_ZERO(&read_fds);
        FD_SET(fileno(stdin), &read_fds);
        tv.tv_sec = 10;
        tv.tv_usec = 0;

        rc = select(nfds, &read_fds, NULL, NULL, &tv);
        if (rc == -1) {
            std::cerr << "select() error: " << strerror(errno) << "\n";
            run = false;
        } else if (rc) {
            if (FD_ISSET(fileno(stdin), &read_fds)) {
                int writeable = buffer->writeable();
                // clamp so we don't woverwrite the whole buffer in one go
                if (writeable > 1024) writeable = 1024;
                writeable = (writeable * sizeof(T)) - read_over;
                read = std::cin.readsome(((char*) buffer->getWritePointer()) + read_over, writeable);
                if (read == 0) {
                    run = false;
                    break;
                }
                buffer->advance((read + read_over) / sizeof(T));
                read_over = (read + read_over) % sizeof(T);
                module->process();
            }
        //} else {
            // no data, just timeout.
        }

        if (std::cin.eof()) {
            run = false;
        }

    }

    delete buffer;
    delete module;
}

AgcCommand::AgcCommand(): Command("agc", "Automatic gain control") {
    add_set("-f,--format", format, {"s16", "float"}, "Data format", true);
    add_set("-p,--profile", profile, {"fast", "slow"}, "AGC profile", true);
    add_option("-a,--attack", attack, "AGC attack rate (slow: 0.1; fast: 0.01)");
    add_option("-d,--decay", decay, "AGC decay rate (slow: 0.0001; fast: 0.001)");
    add_option("-t,--hangtime", hangtime, "AGC hang time (slow: 600; fast: 200)");
    add_option("-m,--max", max_gain, "Maximum gain", true);
    add_option("-i,--initial", initial_gain, "Initial gain", true);
    add_option("-r,--reference", reference, "Reference level", true);
    callback( [this] () {
        if (format == "float") {
            runAgc<float>();
        } else if (format == "s16") {
            runAgc<short>();
        } else {
            std::cerr << "invalid format: " << format << "\n";
        }
    });
}

template <typename T>
void AgcCommand::runAgc() {
    Agc<T>* agc = new Agc<T>();
    if (profile == "fast") {
        agc->setAttack(0.1);
        agc->setDecay(0.001);
        agc->setHangTime(200);
    } else if (profile == "slow") {
        agc->setAttack(0.01);
        agc->setDecay(0.0001);
        agc->setHangTime(600);
    }
    // override profile settings with explicit options on the command-line, if given
    if (attack != 0) agc->setAttack(attack);
    if (decay != 0) agc->setDecay(decay);
    if (hangtime != 0) agc->setHangTime(hangtime);
    agc->setMaxGain(max_gain);
    agc->setInitialGain(initial_gain);
    agc->setReference(reference);
    runModule(agc);
}

FmdemodCommand::FmdemodCommand(): Command("fmdemod", "FM demodulation") {
    callback( [this] () {
        runModule(new FmDemod());
    });
}

AmdemodCommand::AmdemodCommand(): Command("amdemod", "AM demodulation") {
    callback( [this] () {
        runModule(new AmDemod());
    });
}