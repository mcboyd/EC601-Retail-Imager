# Let's start by connecting to VDMS.

import vdms
import os
#print(os.getcwd())

db = vdms.vdms()
db.connect("localhost") # Will connect to localhost on port 55555
blob_arr = []

fd = open("VDMSDataset/dataset/img/surface1.jpg", 'rb')
blob_arr.append(fd.read())
all_queries = []

query = {
          "FindEntity" : {
             "class": "Person",
             "_ref": 3,                     # We set the reference as 3
             "constraints": {
                 "name": ["==", "Tea"]
             }
          }
       }

all_queries.append(query)

props = {}
props["name_file"] = "surface1"
props["type"] = "box"
props["date"] = "October 24rd, 2019"

link = {}
link["ref"] = 3                 # We use the set reference
link["direction"] = "in"
link["class"] = "is_in"

addImage = {}
addImage["properties"] = props
addImage["link"] = link
addImage["format"] = "jpg"      # Format use to store it on VDMS

query = {}
query["AddImage"] = addImage

all_queries.append(query)

print("Query Sent:")
#vdms.aux_print_json(all_queries)
print (all_queries)

response, res_arr = db.query(all_queries, [blob_arr])

#response = json.loads(response)

print("VDMS Response:")
print (db.get_last_response_str())
#vdms.aux_print_json(response)
