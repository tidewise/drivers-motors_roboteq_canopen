#include "Helpers.hpp"
#include <canopen_master/SDO.hpp>
#include <canopen_master/PDO.hpp>
#include <iomanip>

using namespace canopen_master;

void Helpers::ASSERT_QUERIES_SDO_UPLOAD(std::vector<canbus::Message> const& queries,
                                        std::vector<int> const& objects) {
    ASSERT_QUERIES_SDO(SDO_INITIATE_DOMAIN_UPLOAD, queries, objects);
}

void Helpers::ASSERT_QUERIES_SDO_DOWNLOAD(std::vector<canbus::Message> const& queries,
                                          std::vector<int> const& objects) {
    ASSERT_QUERIES_SDO(SDO_INITIATE_DOMAIN_DOWNLOAD, queries, objects);
}

void Helpers::ASSERT_QUERIES_SDO(SDO_COMMANDS command,
                                 std::vector<canbus::Message> const& queries,
                                 std::vector<int> const& objects) {
    ASSERT_EQ(objects.size() / 2, queries.size());

    auto remaining = queries;
    for (size_t i = 0; i < objects.size(); i += 2) {
        int objectID = objects[i];
        int objectSubID = objects[i + 1];

        bool found = false;
        for (auto it = remaining.begin(); it != remaining.end(); ++it) {
            ASSERT_EQ(command, getSDOCommand(*it).command);
            int sdoID = getSDOObjectID(*it);
            int sdoSubID = getSDOObjectSubID(*it);

            found = (sdoID == objectID && sdoSubID == objectSubID);
            if (found) {
                remaining.erase(it);
                break;
            }
        }

        if (!found) {
            std::ostringstream os;
            for (auto const& msg : queries) {
                int sdoID = getSDOObjectID(msg);
                int sdoSubID = getSDOObjectSubID(msg);
                os << "  got: " << std::hex << sdoID << " " << sdoSubID << "\n";
            }

            FAIL()
                << "Expected SDO " << std::hex << objectID << " " << objectSubID
                << " not present,\n" << os.str();
        }
    }

    if (!remaining.empty()) {
        FAIL() << "Found unexpected SDOs";
    }
}

void Helpers::ASSERT_PDO_MAPPINGS(
    std::vector<canopen_master::PDOMapping> const& mappings,
    std::vector<std::vector<int>> const& expected
) {
    ASSERT_EQ(expected.size(), mappings.size());

    for (size_t i = 0; i < mappings.size(); ++i) {
        ASSERT_PDO_MAPPING(mappings[i], expected[i]);
    }
}

void Helpers::ASSERT_PDO_MAPPING(
    canopen_master::PDOMapping const& mappings,
    std::vector<int> const& expected
) {
    ASSERT_EQ(expected.size() / 3, mappings.mappings.size());

    for (size_t i = 0; i < mappings.mappings.size(); ++i) {
        int const* expectedObj = &expected[i * 3];
        PDOMapping::MappedObject const& actualObj = mappings.mappings[i];

        if (expectedObj[0] != actualObj.objectId) {
            FAIL() << "Unexpected object ID at index " << i
                   << " expected " << std::hex << expectedObj[0]
                   << " but got " << (int)actualObj.objectId;
        }
        else if (expectedObj[1] != actualObj.subId) {
            FAIL() << "Unexpected object sub ID at index " << i
                   << " expected " << std::hex << expectedObj[1]
                   << " but got " << (int)actualObj.subId;
        }
        else if (expectedObj[2] != actualObj.size) {
            FAIL() << "Unexpected object size at index " << i
                   << " expected " << std::hex << expectedObj[2]
                   << " but got " << (int)actualObj.size;
        }
    }
}

void Helpers::ASSERT_PDO_MAPPING_MESSAGES(
    std::vector<canbus::Message> const& messages,
    int pdoIndex, bool transmit,
    std::vector<std::vector<int>> const& mappings) {

    auto remaining = messages;
    for (size_t i = 0; i < mappings.size(); ++i) {
        int mappingObjectId = canopen_master::getPDOMappingObjectId(
            transmit, pdoIndex + i
        );
        auto theseMappings = mappings[i];

        while (!remaining.empty()) {
            auto msg = remaining.front();
            remaining.erase(remaining.begin());
            if (getSDOObjectID(msg) != mappingObjectId) {
                continue;
            }

            int subId = getSDOObjectSubID(msg);
            if (subId == 0) {
                continue;
            }

            for (size_t j = 0; j < theseMappings.size(); j += 3) {
                ASSERT_EQ(theseMappings[i + 2], msg.data[0]);
                ASSERT_EQ(theseMappings[i + 1], msg.data[1]);
                ASSERT_EQ(theseMappings[i] & 0xFF, msg.data[2]);
                ASSERT_EQ((theseMappings[i] >> 8) & 0xFF, msg.data[3]);
            }
        }
    }
}
