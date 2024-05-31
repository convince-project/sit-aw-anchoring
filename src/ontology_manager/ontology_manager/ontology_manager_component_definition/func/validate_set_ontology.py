import owlready2 as owlr2
from ontology_manager.ontology_manager_component_definition.func.utils import rename_file
from ontology_manager.ontology_manager_component_definition.func.rulemaker import rulemaker

from ontology_management_msgs.action import ValidateSetOntologyCS

def validate_set_ontology(self, goal_handle, ontology_path, knowledge_domain):
  # Load ontology
  onto = owlr2.get_ontology(ontology_path).load(reload=True)
  
  # Validate ontology & reason on ontology
  try :
    with onto :
      owlr2.sync_reasoner_pellet(onto, keep_tmp_file=True, debug=0)
  except:
    self.get_logger().info('Error: Reasoner failed to validate the ontology')
    message = '^^ Error: Reasoner failed to validate the ontology ^^'
    goal_handle.abort()
    result = ValidateSetOntologyCS.Result()
    result.result.success = False
    result.result.message = message
    return result

  #Clear cache
  onto.destroy()
  # Apply rulemaker
  ontology_path = rulemaker(ontology_path)
  onto = owlr2.get_ontology(ontology_path).load(reload=True)

  # Set - Save ontology
  # if knowledge_domain already exists and ontology is validate,
  kd_ontology_path = ''
  for ontology_dictionnary in self.ontology_list:
    if knowledge_domain in ontology_dictionnary['knowledge_domain']:
      kd_ontology_path = ontology_dictionnary['path']
  
  # if kd_ontology_path exists,
  if kd_ontology_path != '':
#    kd_ontology_path = ontology_path
    # rename the old ontology
    if self.get_parameter('overwriteOntology').get_parameter_value().bool_value == True: 
      log_kd_ontology_path = rename_file(kd_ontology_path)
    # add the path to the dictionnary
    ontology_dictionnary['path'] = kd_ontology_path
    # save the new ontology
    self.get_logger().info(f'Saving ontology at {kd_ontology_path}, old ontology saved at {log_kd_ontology_path}')
    onto.save(file = kd_ontology_path, format = "rdfxml")
  
  # if knowledge_domain doesn't exist and ontology is validate,
  else:
    # save the new ontology
    ontology_path = ontology_path.replace('_design.rdf', f'_runtime.rdf')
    self.ontology_list.append({'knowledge_domain': knowledge_domain, 'path': ontology_path})
    self.get_logger().info(f'Saving ontology at {ontology_path}')
    onto.save(file = ontology_path, format = "rdfxml")

  #Clear cache
  onto.destroy()

  # Outputs
  message = '^^ Done validate_set_ontology ^^'

  return self.ontology_list, message

