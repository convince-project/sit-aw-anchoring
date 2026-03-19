#include "typedb_client.hpp"

bool TypeDBClient::create_or_replace_database(
  const std::unique_ptr<TypeDB::Driver>& driver, const std::string &dbName, const bool dbReset) {

  if (driver->databases.contains(dbName)) {
    if (dbReset) {
      // replace database
      driver->databases.get(dbName).deleteDatabase(); // Delete the database if it exists already
      driver->databases.create(dbName); // Create a new database
    }
    else {
      // database exists and no reset option, don't do delete, don't create
      return false;
    }
  } else {
    // create database
    driver->databases.create(dbName);
  }
  return true;

}

void TypeDBClient::write_schema(
  const std::string& schema, const std::unique_ptr<TypeDB::Driver>& driver, const std::string &dbName) {
    TypeDB::Options options;

    auto session = driver->session(dbName, TypeDB::SessionType::SCHEMA, options);
    auto tx      = session.transaction(TypeDB::TransactionType::WRITE, options);

    tx.query.define(schema, options).get();

    tx.commit();
    tx.close();
}

void TypeDBClient::insert_data(
  const std::vector<std::string>& queries, const std::unique_ptr<TypeDB::Driver>& driver, const std::string &dbName) {
    TypeDB::Options options;

    auto session = driver->session(dbName, TypeDB::SessionType::DATA, options);
    auto tx = session.transaction(TypeDB::TransactionType::WRITE, options);

    for (std::string q : queries) {
      tx.query.insert(q, options);
    }

    tx.commit();
    tx.close();
}

void TypeDBClient::update_data(
  const std::vector<std::string>& queries, const std::unique_ptr<TypeDB::Driver>& driver, const std::string &dbName) {
    TypeDB::Options options;

    auto session = driver->session(dbName, TypeDB::SessionType::DATA, options);
    auto tx = session.transaction(TypeDB::TransactionType::WRITE, options);

    for (std::string q : queries) {
      tx.query.update(q, options);
    }

    tx.commit();
    tx.close();
}

std::vector<TypeDB::ConceptMap> TypeDBClient::read_data(
  const std::string& query, const std::unique_ptr<TypeDB::Driver>& driver, const std::string &dbName) {
    TypeDB::Options options;

    auto session = driver->session(dbName, TypeDB::SessionType::DATA, options);
    auto tx = session.transaction(TypeDB::TransactionType::READ, options);

    auto results_ = tx.query.get(query, options);

    // return
    std::vector<TypeDB::ConceptMap> results;
    for (TypeDB::ConceptMap& r : results_)
    {
      results.push_back(std::move(r));
    }

    tx.close();

    return results;
}
