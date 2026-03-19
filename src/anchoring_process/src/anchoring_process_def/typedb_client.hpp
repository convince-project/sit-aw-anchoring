#include <vector>
#include <string>
#include <typedb_driver.hpp>

class TypeDBClient
{
public:

    static bool create_or_replace_database(
      const std::unique_ptr<TypeDB::Driver>& driver, const std::string &dbName, const bool dbReset);

    static void write_schema(
      const std::string& schema, const std::unique_ptr<TypeDB::Driver>& driver, const std::string &dbName);

    static void insert_data(
      const std::vector<std::string>& queries, const std::unique_ptr<TypeDB::Driver>& driver, const std::string &dbName);

    static void update_data(
      const std::vector<std::string>& queries, const std::unique_ptr<TypeDB::Driver>& driver, const std::string &dbName);

    static std::vector<TypeDB::ConceptMap> read_data(
      const std::string& query, const std::unique_ptr<TypeDB::Driver>& driver, const std::string &dbName);

};
