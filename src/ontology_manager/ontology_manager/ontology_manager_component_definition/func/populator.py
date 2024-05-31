# Init
import json
import owlready2 as owlr2

def populator(onto_path, setup_populator_path, DT_config_path, debug = 0):

    # Loading Ontology
    onto = owlr2.get_ontology(onto_path).load(reload=True)

    ### -------Add Digital Twin configurations ------- ###

    #Add Digital Twin configurations 
    # Load the Digital Twin configurations JSON file

    VERSION_XDE_CONF = "0.3"

    with open(DT_config_path) as json_file:
        json_data_DT = json.load(json_file)
        if json_data_DT["version"] != VERSION_XDE_CONF:
            print("Wrong version of JSON file")
            exit()
        else:
            data = json_data_DT["data"]

    # Manipulation of Ontology -> with onto:
    # Add Digital Twin configurations to the ontology : Digital Twin (name), Alias, isRealTime
    with onto:
        for Dt_individual in data:
            data_DT = Dt_individual["data"]
            name_DT = Dt_individual["Digital_Twin"]
            # Add Digital Twin to the ontology
            tmp_individual_onto = onto.DigitalTwin(name_DT)

            for DT_property in data_DT:
                # print(f"DT : {DT_property}")
                tmp_individual_onto = onto.Alias(DT_property["name"])
                tmp_individual_onto_isAliasOf = onto.search(iri = "*" + DT_property["ontology_name"])
                tmp_individual_onto.isAliasOf.append(tmp_individual_onto_isAliasOf[0])
                tmp_individual_onto.isRealTime.append(bool(DT_property["real_time"]))
                tmp_individual_onto.fromDigitalTwin.append(onto.DigitalTwin(name_DT))

    ### -------Add Digital Twin Indivials population (setup) ------- ###

    # Loading JSON setup file

    VERSION_SETUPFILE = "0.2"

    with open(setup_populator_path) as json_file:
        json_data = json.load(json_file)
        if json_data["version"] != VERSION_SETUPFILE:
            print("Wrong version of JSON file")
            exit()
        else:
            data = json_data["data"]

    # Manipulation of Ontology
    # Add setup populators to the ontology : Add individuals to the ontology
    with onto:
        for individual in data:
            individual_id = individual["id"]
            individual_name = individual["name"]
            # individual_class = "onto." + str(individual["setup_properties"]["class"])
            individual_class = individual["setup_properties"]["class"]
            
            print(f"Id \033[92m {individual_id} \033[0m - Creating \033[92m {individual_name} \033[0m of class \033[92m {individual_class} \033[0m")

            #create the individual
            if individual_class == "Color":
                tmp_individual_onto = onto.Color(individual_id)
            elif individual_class == "Cube":
                for i in range(len(individual["DT_config"])):
                    tmp_individual_onto_ID = onto.Id(individual["DT_config"][i]["id_DT"])
                    tmp_individual_onto = onto.Cube(individual_id)
                    tmp_individual_onto.hasId.append(tmp_individual_onto_ID)
                    tmp_individual_onto.fromDigitalTwin.append(onto.DigitalTwin(individual["DT_config"][i]["DigitalTwin"]))
                tmp_individual_onto.hasColor.append(onto.Color(individual["setup_properties"]["color"]))
            elif individual_class == "Stand":
                for i in range(len(individual["DT_config"])):
                    tmp_individual_onto_ID = onto.Id(individual["DT_config"][i]["id_DT"])
                    tmp_individual_onto = onto.Stand(individual_id)
                    tmp_individual_onto.hasId.append(tmp_individual_onto_ID)
                    tmp_individual_onto.fromDigitalTwin.append(onto.DigitalTwin(individual["DT_config"][i]["DigitalTwin"]))
            elif individual_class == "Table":
                for i in range(len(individual["DT_config"])): 
                    tmp_individual_onto_ID = onto.Id(individual["DT_config"][i]["id_DT"])
                    tmp_individual_onto = onto.Table(individual_id)
                    tmp_individual_onto.hasId.append(tmp_individual_onto_ID)
                    tmp_individual_onto.fromDigitalTwin.append(onto.DigitalTwin(individual["DT_config"][i]["DigitalTwin"]))
            elif individual_class == "UniversalRobot":
                for i in range(len(individual["DT_config"])):
                    tmp_individual_onto_ID = onto.Id(individual["DT_config"][i]["id_DT"])
                    tmp_individual_onto = onto.UniversalRobot(individual_id)
                    tmp_individual_onto.hasId.append(tmp_individual_onto_ID)
                    tmp_individual_onto.fromDigitalTwin.append(onto.DigitalTwin(individual["DT_config"][i]["DigitalTwin"]))
                tmp_individual_onto.hasRangeValue.append(float(individual["setup_properties"]["range"]))
            elif individual_class == "Gripper":
                for i in range(len(individual["DT_config"])):
                    tmp_individual_onto_ID = onto.Id(individual["DT_config"][i]["id_DT"])
                    tmp_individual_onto = onto.Gripper(individual_id)
                    tmp_individual_onto.hasId.append(tmp_individual_onto_ID)
                    tmp_individual_onto.fromDigitalTwin.append(onto.DigitalTwin(individual["DT_config"][i]["DigitalTwin"]))    
                tmp_individual_onto.IsComponentOf.append(onto.UniversalRobot(individual["setup_properties"]["isComponentOf"]))


        ### -------Save and reasonning------- ###
        if debug >= 1:
            # Save ontology
            onto_path_debug = onto_path.split(".")[0] + "_populated" + ".rdf"
            onto.save(file = onto_path_debug, format = "rdfxml")

        # Reasonning
        # sync_reasoner_pellet(onto, keep_tmp_file=True, debug=0)

    if debug >= 1:
        onto_path_debug = onto_path_debug.split(".")[0] + "_reasoned" + ".rdf"
        onto.save(file = onto_path_debug, format = "rdfxml")
    
    # Save the ontology
    onto.save(file = onto_path, format = "rdfxml")

    #Clear cache
    onto.destroy()

    return onto_path
