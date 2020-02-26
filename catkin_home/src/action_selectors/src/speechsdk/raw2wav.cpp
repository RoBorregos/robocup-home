#include <iostream>
#include <cstdlib>
#include <cerrno>
#include <fstream>
#include <vector>

const unsigned long long DEFAULT_DURATION = 30u;
const unsigned long long DEFAULT_REFRESH = 16000u;

int main(int argc, char **argv) {
	if (argc < 2) {
		std::cerr << "Usage: " << argv[0]
		          << " <filename> [<duration>] [<refresh>]\n"
		             "(default: " << DEFAULT_DURATION << " sec, "
		          << DEFAULT_REFRESH << " Hz)\n";
		std::exit(1);
	}

	unsigned long long duration = DEFAULT_DURATION;
	if (argc >= 3) {
		duration = ::strtoull(argv[2], nullptr, 10);
		if (errno == ERANGE) {
			std::cerr << "Error: duration out of range\n";
			std::exit(1);
		}
	}

	unsigned long long refresh = DEFAULT_REFRESH;
	if (argc >= 4) {
		refresh = ::strtoull(argv[3], nullptr, 10);
		if (errno == ERANGE) {
			std::cerr << "Error: refresh rate out of range\n";
			std::exit(1);
		}
	}

	std::ofstream out(argv[1], std::ios::binary);
	if (!out) {
		std::cerr << "Error: unable to open output file\n";
		std::exit(1);
	}

	const auto pushInt = [&out] (int x) {
		out.put(static_cast<char>(x));
		out.put(static_cast<char>(x >> 8));
		out.put(static_cast<char>(x >> 16));
		out.put(static_cast<char>(x >> 24));
	};

	unsigned long long samples = duration * refresh;
	std::vector<char> buf;
	for (unsigned long long i = 0; i < samples; ++i) {
		int ch = std::cin.get();
		if (ch == -1) {
			samples = i + 1;
			break;
		}
		buf.push_back(static_cast<char>(ch));
	}

	pushInt(0x46464952); // RIFF
	pushInt(static_cast<int>(samples) + 0x24);
	pushInt(0x45564157); // WAVE
	pushInt(0x20746D66); // fmt_
	pushInt(0x00000010);
	pushInt(0x00010001);
	pushInt(refresh);
	pushInt(refresh);
	pushInt(0x00080001);
	pushInt(0x61746164); // data
	pushInt(static_cast<int>(samples));
	out.write(buf.data(), samples);
}
