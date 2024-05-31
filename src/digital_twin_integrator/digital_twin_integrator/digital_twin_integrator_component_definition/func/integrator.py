# Init
from owlready2 import *
import json

def integrator(onto_path, DT_Data_path, debug = 0):

    # Loading Ontology

    onto = get_ontology(onto_path).load(reload=True)

    # Loading DT_Data JSON

    with open(DT_Data_path) as json_file:
        json_data = json.load(json_file)

    # Manipulation of Ontology -> with onto:
    # Integrating DT_Data into Ontology
    with onto:
        for i in onto.individuals():
            if i.fromDigitalTwin:
                fromDigitalTwin = i.fromDigitalTwin[0].name
                if fromDigitalTwin == "XDE":
                    if i.isAliasOf:
                        isAliasOf_name = i.isAliasOf[0].name
                        isAliasOf_range = i.isAliasOf[0].get_range_iri()[0].split("#")[1]
                        for dt_ind in json_data:
                            for key in dt_ind:
                                if i.name == key:
                                    ind = getattr(onto, dt_ind['dt_id']).isIdOf[0]

                                    print(f"for id : {dt_ind['dt_id']} id of {ind.name} and key : {key} as alias of {isAliasOf_name} in the ontology, the value is {dt_ind[key]}")
                                   
                                    if isAliasOf_range == "float" or range =="double" or isAliasOf_range == "int" or isAliasOf_range =="decimal":
                                        dt_ind_key = float(dt_ind[key])
                                    elif isAliasOf_range == "string":
                                        dt_ind_key = str(dt_ind[key])
                                    elif isAliasOf_range == "bool":
                                        dt_ind_key = bool(dt_ind[key])
                                    else: #Object property -> dt_ind[key] is the id of an individual
                                        dt_ind_key = getattr(onto, dt_ind[key]).isIdOf[0]

                                    
                                    #if dataproperty is functional
                                    print(f"{ind.name}, {isAliasOf_name} getattr : {getattr(ind, isAliasOf_name)}, ")
                                    if isinstance(getattr(ind, isAliasOf_name), list) == False:
                                        # print(f"{ind.name}, {isAliasOf_name} is functionnal : {getattr(ind, isAliasOf_name)}")
                                        setattr(ind, isAliasOf_name, dt_ind_key)
                                    else :
                                        # print(f"{ind.name}, {isAliasOf_name} is not functionnal : {getattr(ind, isAliasOf_name)}")
                                        if getattr(ind, isAliasOf_name):
                                            getattr(ind, isAliasOf_name)[0] = dt_ind_key
                                        else:
                                            getattr(ind, isAliasOf_name).append(dt_ind_key)
                                    
                                    print(f"get {isAliasOf_name} : {getattr(ind, isAliasOf_name)}")
        
        if debug >= 1:
            # Save ontology
            onto_path_debug = onto_path.split(".")[0] + "_integrated" + ".rdf"
            onto.save(file = onto_path_debug, format = "rdfxml")

        # Reasonning
        sync_reasoner_pellet(onto, keep_tmp_file=True, debug=0)

    if debug >= 1:
        # Save ontology
        onto_path_debug = onto_path_debug.split(".")[0] + "_reasoned" + ".rdf"
        onto.save(file = onto_path_debug, format = "rdfxml")
    
    onto.save(file = onto_path, format = "rdfxml")

    #Clear cache
    onto.destroy()

    return onto_path


# if key == "pose":
#     # equivalent_name = onto.pose.equivalent_to
#     onto.Id(dt_ind["dt_id"]).hasPose.append(str(dt_ind[key]))
# elif key == "bounding_box_height":
#     onto.Id(dt_ind["dt_id"]).hasHeight.append(float(dt_ind[key]))
# elif key == "bounding_box_width":
#     onto.Id(dt_ind["dt_id"]).hasWidth.append(float(dt_ind[key]))
# elif key == "bounding_box_depth":
#     onto.Id(dt_ind["dt_id"]).hasDepth.append(float(dt_ind[key]))
