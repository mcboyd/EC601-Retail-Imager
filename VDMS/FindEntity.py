import vdms
import util
import os

db = vdms.vdms()
db.connect("localhost") # Will connect to localhost on port 55555

query = """
[
   {
      "FindEntity" : {
         "class" : "Person", 
         "constraints" : {
		        "name": [ "==", "Tea" ]
		    },
         "results" : {
            "list" : [ "name", "lastname"]
         }
      }
   }
]   
"""

query1 = """
[
   {
      "FindConnection": {
		    "class": "is_in",

		    "results": {
		        "count" : []
		    }
		}
   }
]   
"""

query2 = """
[
   {
      "FindImage": {
      		"_ref": 344554,
		    "constraints" : {
		        "date": [ "==", "October 24rd, 2019" ]
		    },
		   "results": {
		        "list": [ "name_file", "type", "date" ]
		    }
		}
   }
]   
"""

# Below works to get the test files inserted on test person!
query3 = """
[
	{
		"FindEntity": {
		    "_ref": 344554,
		    "class": "Person",
		    "constraints" : {
		        "name": [ "==", "Tea" ]
		    }
		}
	},
	{
		"FindImage": {
		    "link": {
		       "ref": 344554
		   },
		   "constraints" : {
		        "date": [ "==", "October 24rd, 2019" ]
		    },
		   "results": {
		        "list": [ "name_file", "type", "date" ]
		    }
		}
		
	}
]
"""

query4 = """
[
   {
      "FindDescriptor" : {
         "set": "hike_mt_rainier", 
         "results": {
             "list": ["_distance", "_id", "_label"] 
         }
      }
   }
]   
"""

response, images = db.query(query2)
print (db.get_last_response_str())
#util.display_images(images)    
#print ("Number of images:", len(images))