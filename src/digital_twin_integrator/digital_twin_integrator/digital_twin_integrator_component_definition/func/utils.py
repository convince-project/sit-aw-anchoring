import owlready2 as owlr2
import numpy as np

def get_request_compute(onto_path):    
    onto = owlr2.get_ontology(onto_path).load(reload=True)

    # Querry to get the [[subject, object], [subject, object], ...] of the relation hasGraspabilityRequestCompute
    with onto:
      hasGraspabilityRequestCompute_Sub_Obj = np.array(list(onto.hasGraspabilityRequestCompute.get_relations()))
      
      # Create a matrix with the relation hasGraspabilityRequestCompute in the form [[hasGraspabilityRequestCompute, subject, object], [hasGraspabilityRequestCompute, subject, object], ...]
      hasGraspabilityRequestCompute = np.zeros([len(hasGraspabilityRequestCompute_Sub_Obj), 3], dtype=object)
      hasGraspabilityRequestCompute[:, 0] = onto.hasGraspabilityRequestCompute.name
      hasGraspabilityRequestCompute[:, 1] = [ind.hasId[0].name for ind in hasGraspabilityRequestCompute_Sub_Obj[:,0]]
      hasGraspabilityRequestCompute[:, 2] = [ind.hasId[0].name for ind in hasGraspabilityRequestCompute_Sub_Obj[:,1]]
    
    return hasGraspabilityRequestCompute
