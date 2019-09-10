#pragma once
#include <vector>
#include <iostream>
#include <fstream>
#include <limits>
#include <Eigen/Eigen>

namespace fab_translation {

	typedef unsigned int uint;

    class GCodeConverter{

    private:
        static void WriteGCodeHeader(std::ofstream& gcode_filestream){
            gcode_filestream << "G28 ;\n"; //home head
            gcode_filestream << "G90 ;\n"; //absolute positioning mode
            gcode_filestream << "M106 S255 ;\n"; //fan on!
            gcode_filestream << "M109 S190 T0 ;\n"; //heat head to 190 degrees, then wait until it reaches that point.
            gcode_filestream << "M190 S50 ;\n"; //heat bed to 50 degrees, if possible, and wait
        }

        static void WriteGCodeFooter(std::ofstream& gcode_filestream) {
            gcode_filestream << "M104 S0 ;\n"; //cool hot end
            gcode_filestream << "M140 S0 ; \n"; //cool printbed
            gcode_filestream << "M107 ; \n"; //fan off
            gcode_filestream << "G1 X0 Y180 F9000; \n"; //move the bed for easy retrieval
        }

        static void MoveToPosition(std::ofstream& gcode_filestream, const double speed, const Eigen::Vector3d& target){
            //TODO: some sort of string builder for efficiency
            std::string code = "G0 X";
            code.append(std::to_string(target.x()));
            code.append(" Y");
            code.append(std::to_string(target.y()));
            code.append(" Z");
            code.append(std::to_string(target.z()));
            code.append(" F");
            code.append(std::to_string(speed));
            code.append(" ;\n");
            gcode_filestream << code;
        }

        static double WriteExtrudeSingleSegment(std::ofstream& gcode_filestream, const double speed, const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, double extrude_start){
            //TODO: modularize above code and this code somehow
            std::string code = "G1 X";

            //calculate extrude amount:
            double extrude_amt = (p1 - p0).norm() * .05;

            code.append(std::to_string(p1.x()));
            code.append(" Y");
            code.append(std::to_string(p1.y()));
            code.append(" Z");
            code.append(std::to_string(p1.z()));
            
            code.append(" E");
            code.append(std::to_string(extrude_amt + extrude_start));
            
            code.append(" F");
            code.append(std::to_string(speed));
            
            code.append(" ;\n");
            gcode_filestream << code;
            return extrude_amt;
        }

        static double WriteExtrudeSegment(std::ofstream& gcode_filestream, const double speed, const std::vector<Eigen::Vector3d>& targets, double extrude_start){

            //assume already at starting point.

            Eigen::Vector3d last_target = targets[0];
            for (uint i = 1; i < targets.size(); ++i){
                Eigen::Vector3d target = targets[i];
                extrude_start += WriteExtrudeSingleSegment(gcode_filestream, speed, last_target, target, extrude_start);
                last_target = target;
            }
            gcode_filestream << "G92 E0;\n";
            return 0;
        }


    public:

		static void ConvertToGCode(const std::string & output_file_name,
			const std::vector<std::vector<std::vector<Eigen::Vector3d>>> & contours,
			const std::vector<std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>> & infill_edges)
		{
			std::cout << "Writing GCode to file: " << output_file_name << std::endl;
			const double default_speed = 600; // 600 mm / min or 1cm per second
			double extruded_amount = 1;

			std::ofstream gcode_file;
			gcode_file.open(output_file_name.c_str());
			WriteGCodeHeader(gcode_file);

			// We should have the same number of infill layers as contour layers
			assert(contours.size() == infill_edges.size());

			for (int layer_num = 0; layer_num < contours.size(); ++layer_num) {
				auto& contour_layer = contours[layer_num];
				auto& infill_layer = infill_edges[layer_num];

				for (auto& contour : contour_layer) {
					if (contour.size() > 0) {
						MoveToPosition(gcode_file, default_speed, contour.front());
						extruded_amount = WriteExtrudeSegment(gcode_file, default_speed, contour, extruded_amount);
						extruded_amount = WriteExtrudeSingleSegment(gcode_file, default_speed, contour.back(), contour.front(), extruded_amount);
					}
				}
				
				if (infill_layer.size() > 0) {
					// Reset extruder position
					gcode_file << "G92 E0;\n";
					extruded_amount = 0;
					MoveToPosition(gcode_file, default_speed, infill_layer.front().first);
					auto last_position = infill_layer.front().first;
					for (auto& edge : infill_layer) {
						if (edge.first != last_position) {
							MoveToPosition(gcode_file, default_speed, edge.first);
						}
						extruded_amount += WriteExtrudeSingleSegment(gcode_file, default_speed, edge.first, edge.second, extruded_amount);
						last_position = edge.second;
					}
				}
			}

			WriteGCodeFooter(gcode_file);
			gcode_file.close();
		}

        static void ConvertToGCode(const std::vector<std::vector<Eigen::Vector3d>>& segments, const std::string & output_file_name){
            std::cout << "Writing GCode to file: " << output_file_name << std::endl;
            const double default_speed = 600; //600 mm / min or 1cm per second
            double extruded_amount = 1;
            //write to gcode

            std::ofstream gcode_file;
            gcode_file.open (output_file_name.c_str());
            WriteGCodeHeader(gcode_file);
            for (std::vector<Eigen::Vector3d> segment : segments){
                MoveToPosition(gcode_file, default_speed, segment[0]);
                extruded_amount = WriteExtrudeSegment(gcode_file, default_speed, segment, extruded_amount);
            }

            WriteGCodeFooter(gcode_file);
            gcode_file.close();
        }

    };

}
