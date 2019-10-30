import pcl

cloud_in = pcl.load("hand.ply")
pcl.save(cloud_in, "hand.pcd")