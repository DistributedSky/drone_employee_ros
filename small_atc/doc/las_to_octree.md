## Making the OctoMap by LASer data

The first - get LASer data from opentopography.org.

You need to install `liblas` and `octomap` libraries and tools.

    las2las --offset "-569807.28 -4108300.46 -423.55"  --output points.las downloads/points.las

Offset data for centering future map.

    las2txt --stdout --parse xyz --delimiter " " points.las > points.txt

Serialize binary las to text points.

    echo NODE 0 0 300 0 0 0 > header.txt
    cat header.txt points.txt > ocdata.txt

Prepare laser scan data for octree creation.

     log2graph ocdata.txt ocdata.graph
     graph2tree -res 3 -i ocdata.graph -o ocdata.bt

Choose resolution by `-res` key.
