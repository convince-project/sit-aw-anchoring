from owlready2 import *
import numpy as np
from ontology_manager.ontology_manager_component_definition.func.utils import ros_pose_to_euclidean

def rulemaker(onto_path, debug = 0):
    # Loading Ontology
    onto = get_ontology(onto_path).load(reload=True)
    
    ### ----- isInRangeOf rule ----- ###
    # isInRangeOf = distance(agent, physicalObject) < range(agent)

    # Manipulation of Ontology
    with onto:
        list_of_onto_range_values_relations = list(onto.hasRangeValue.get_relations())
        if list_of_onto_range_values_relations:
          ind_hasRange = np.array(list_of_onto_range_values_relations)[:,0]
          list_of_pose_relations = list(onto.hasPose.get_relations())
          if list_of_pose_relations:
            ind_hasPose = np.array(list_of_pose_relations)[:,0]
            for i_hasRange in ind_hasRange:
                i_hasRange_Position = ros_pose_to_euclidean(i_hasRange.hasPose)
                i_hasRange_Range = i_hasRange.hasRangeValue[0]
                for i_hasPose in ind_hasPose:
                    if i_hasPose.name != i_hasRange.name:
                        i_hasPose_Position = ros_pose_to_euclidean(i_hasPose.hasPose)
                        distance = np.linalg.norm(i_hasRange_Position - i_hasPose_Position)
                        if distance < i_hasRange_Range:
                            i_hasPose.isInRangeOf.append(i_hasRange)
                            # print(f"{i_hasPose.name} is in range of {i_hasRange.name}")
                        else:
                            if i_hasRange in i_hasPose.isInRangeOf:
                                i_hasPose.isInRangeOf.remove(i_hasRange)

        # sync_reasoner_pellet(onto, keep_tmp_file=True)
            # Rule below does not need reasoning -> Reasoning commented out

        ### ----- isTouching rule ----- ###
        # isTouching = x/y/z of the bounding box of the object is touching the x/y/z of the bounding box of on other object
        
        TOLERANCE = 0.01 #meter
        list_of_width_relations = list(onto.hasWidth.get_relations())
        if list_of_width_relations:
          ind_hasBoundingBox = np.array(list_of_width_relations)[:,0]
          for i_hasBD in ind_hasBoundingBox:
              if i_hasBD.hasPose:
                  i_hasBD_Width = i_hasBD.hasWidth[0]
                  i_hasBD_Height = i_hasBD.hasHeight[0]
                  i_hasBD_Length = i_hasBD.hasLength[0]
                  i_hasBD_Position = ros_pose_to_euclidean(i_hasBD.hasPose)
                  for j_hasBD in ind_hasBoundingBox:
                      if j_hasBD.hasPose:
                          j_hasBD_Width = j_hasBD.hasWidth[0]
                          j_hasBD_Height = j_hasBD.hasHeight[0]
                          j_hasBD_Length = j_hasBD.hasLength[0]
                          j_hasBD_Position = ros_pose_to_euclidean(j_hasBD.hasPose)
                          if (i_hasBD.name == j_hasBD.name):
                              continue # Skip if same
                          else:
                              if (abs(i_hasBD_Position[0][0] - j_hasBD_Position[0][0]) < i_hasBD_Length/2 + j_hasBD_Length/2 + TOLERANCE) \
                                  and (abs(i_hasBD_Position[0][1] - j_hasBD_Position[0][1]) < i_hasBD_Width/2 + j_hasBD_Width/2 + TOLERANCE) \
                                  and (abs(i_hasBD_Position[0][2] - j_hasBD_Position[0][2]) < i_hasBD_Height/2 + j_hasBD_Height/2 + TOLERANCE):
                                          # print(f"{i_hasBD.name} is touching {j_hasBD.name}")
                                          i_hasBD.isTouching.append(j_hasBD)
                              else : 
                                  if j_hasBD in i_hasBD.isTouching:
                                      i_hasBD.isTouching.remove(j_hasBD)

        
        # sync_reasoner_pellet(onto, keep_tmp_file=True) 
            # Rule below does not need reasoning -> Reasoning commented out

        ### ----- isAboveArea rule ----- ###
        # isAboveArea = x/y/z of the bounding box of the object is above the x/y/z of the bounding box of on other object
        
        # ind_hasBoundingBox = np.array(list(onto.hasWidth.get_relations()))[:,0] #Already defined above

        list_of_width_relations = list(onto.hasWidth.get_relations())
        if list_of_width_relations:
          ind_hasBoundingBox = np.array(list_of_width_relations)[:,0]
          for i_hasBD in ind_hasBoundingBox:
              if i_hasBD.hasPose:
                  i_hasBD_Width = i_hasBD.hasWidth[0]
                  i_hasBD_Height = i_hasBD.hasHeight[0]
                  i_hasBD_Length = i_hasBD.hasLength[0]
                  i_hasBD_Position = ros_pose_to_euclidean(i_hasBD.hasPose)
                  for j_hasBD in ind_hasBoundingBox:
                      if j_hasBD.hasPose:
                          j_hasBD_Width = j_hasBD.hasWidth[0]
                          j_hasBD_Height = j_hasBD.hasHeight[0]
                          j_hasBD_Length = j_hasBD.hasLength[0]
                          j_hasBD_Position = ros_pose_to_euclidean(j_hasBD.hasPose)
                          if (i_hasBD.name == j_hasBD.name):
                              continue # Skip if same
                          else: 
                              #i is above j
                              if (i_hasBD_Position[0][2] > j_hasBD_Position[0][2] + j_hasBD_Height/2 + TOLERANCE) \
                                  and (abs(i_hasBD_Position[0][0] - j_hasBD_Position[0][0]) < i_hasBD_Length/2 + j_hasBD_Length/2 + TOLERANCE) \
                                  and (abs(i_hasBD_Position[0][1] - j_hasBD_Position[0][1]) < i_hasBD_Width/2 + j_hasBD_Width/2 + TOLERANCE):
                                          # print(f"{i_hasBD.name} is above {j_hasBD.name}")
                                          i_hasBD.isAboveArea.append(j_hasBD)
                              else :
                                  if j_hasBD in i_hasBD.isAboveArea:
                                      i_hasBD.isAboveArea.remove(j_hasBD)

          sync_reasoner_pellet(onto, infer_property_values=True, keep_tmp_file=True, debug=0) # Rule below needs reasoning
            # infer_property_values=True # in order to apply SWRL rule
        
        ### ----- isOnTopOf rule ----- ###
        # isOnTopOf = isAboveArea + isTouching
        # In Anchology (SWRL)
            
        ### ----- hasEmptyTopSurface rule ----- ###
        # hasEmptyTopSurface = nothing is on top of the object

        functional_part_instances = onto.FunctionalPart.instances()
        physical_bodies = onto.PhysicalBody.instances()
        for pb in physical_bodies:
          if pb in functional_part_instances:
            physical_bodies.remove(pb)
        for PO1 in physical_bodies: # non-functional physical bodies
            for PO2 in physical_bodies: # non-functional physical bodies
                if PO1 != PO2: # useless to compare with itself
                  if PO2.isOntopOf:
                      if PO1.name == PO2.isOntopOf[0].name: #Not empty
                          if PO1.hasEmptyTopSurface:
                              PO1.hasEmptyTopSurface.remove(PO1.hasEmptyTopSurface[0])
#                              print(f"{PO1.name} no longer has empty top surface")
                          print(f"PO1 {PO1.name} wont have an empty top surface!")
                          break
                      else: #Empty
                          PO1.hasEmptyTopSurface.append(PO1)
                          print(f"{PO1.name} has empty top surface")

        sync_reasoner_pellet(onto, infer_property_values=True, keep_tmp_file=True, debug=0) # infer_property_values=True # in order to apply SWRL rule
        # eventually in future: infer_data_property_values = True

    ### ----- isGraspableRequestCompute rule ----- ###
    # isGraspableRequestCompute = isInRangeOf + isEmpty + isOnTopOf Table 
    # In Anchology (SWRL)
    
    if debug >= 1:
        onto_path_debug = onto_path.split(".")[0] + "_ruled" + ".rdf"
        onto.save(file = onto_path_debug, format = "rdfxml")
    else:
        onto.save(file = onto_path, format = "rdfxml")

    #Clear cache
    onto.destroy()

    return onto_path
