dirs=launch/ data/ "config" "3dof_platform_driver" "aubo_manipulator_driver" "coverage_path_planning" "mobile_platform_driver" "painting_opreating_planning" "plc_package"
for dir in ${dirs}
    do
	cp -vf README.md $(dir)
    done
