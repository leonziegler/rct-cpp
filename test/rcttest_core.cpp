#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <rsc/logging/LoggerFactory.h>

#include <rsb/converter/converters.h>

using namespace testing;

int main(int argc, char* argv[]) {

    srand(time(NULL));
    rsc::logging::LoggerFactory::getInstance().reconfigure(
                rsc::logging::Logger::LEVEL_TRACE);
    rsb::converter::registerDefaultConverters();

    InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();

}
