# Documentation for the `setupfile.json`

Here is each possible field of the `setupfile.json` file.

```json
{
    "version" : "*.*", 
    "data" : 
        [
            // First individual
            {
                "id" : "id of individual choose by user",
                "name" : "name of individual choose by user",
                
                //objects and data properties of the individual that are not characterize  by the DT
                "setup_property" : 
                    {
                        "classe" : "class of the individual",
                        "isComponentOf" : "id of the component",
                        "color" : "color of the individual"
                    },
                
                //DTs of the individual
                "DT_config" :
                    [
                        {
                            "DigitalTwin" : "name of the DT",
                            "id_DT" : "id of the individual in the DT"
                        }
                    ]
            },
            // Other individuals
    
        ]
}
```
