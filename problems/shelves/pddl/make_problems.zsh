#! /usr/bin/zsh

for i in {5..20}
do
  cp "pb_$(( i - 1 )).pddl" "pb_${i}.pddl" 
  sed -i -e "s/shelves_pb_$(( i - 1 ))/shelves_pb_${i}/" -e "s/-objs/red_stick_$(( i - 2 )) -objs/" "pb_${i}.pddl"
  if (( $i % 2 == 0 ))
  then
    sed -i -e "s/(on-upper-shelf red_stick_$(( i - 3 )))/(on-upper-shelf red_stick_$(( i - 3 ))) (on-lower-shelf red_stick_$(( i - 2 )))/" "pb_${i}.pddl"
  else
    sed -i -e "s/(on-lower-shelf red_stick_$(( i - 3 )))/(on-lower-shelf red_stick_$(( i - 3 ))) (on-upper-shelf red_stick_$(( i - 2 )))/" "pb_${i}.pddl"
  fi
done
