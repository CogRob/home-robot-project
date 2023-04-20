#!/bin/bash
container_count=$(docker ps | wc -l | awk '{print $1-1}')
if [ $container_count -eq 1 ]; then
	last_container_id=$(docker ps --format "{{.ID}}" | head -n 1)
	docker exec -it $last_container_id /bin/bash
else
	echo "you have $container_count container, you should have only one!!"
fi
