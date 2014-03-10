#include "passage_finder.h"




Passage_finder::Passage_finder (Line_param &line)
{
	const double critical_angle = 11.0;
	double ref_angle = atan(line.kin_inliers.at(0).x/line.kin_inliers.at(0).y)*180/PI;
	for (int i = 1; i < line.kin_inliers.size(); i++)
	{
		double current_angle = atan(line.kin_inliers.at(i).x/line.kin_inliers.at(i).y)*180/PI;
		if (fabs(current_angle - ref_angle) > critical_angle) this->add_passage (i-1, i, line);
		ref_angle = current_angle;
	}
	this->check_boundary(line);
};


void Passage_finder::add_passage (int id1, int id2, Line_param &line)
{
	Passage new_passage;

	new_passage.width += pow(line.kin_inliers.at(id1).x - line.kin_inliers.at(id2).x, 2.0);
	new_passage.width += pow(line.kin_inliers.at(id1).y - line.kin_inliers.at(id2).y, 2.0);
	new_passage.width =  sqrt(new_passage.width);

	new_passage.kin_middle.x = (line.kin_inliers.at(id1).x + line.kin_inliers.at(id2).x)/2;
	new_passage.kin_middle.y = (line.kin_inliers.at(id1).y + line.kin_inliers.at(id2).y)/2;

	if (new_passage.width > 1.7)
		this->passage.push_back(new_passage);
};


void Passage_finder::check_boundary (Line_param &line)
{
	double left_ref_angle = atan(line.kin_inliers.at(0).x/line.kin_inliers.at(0).y)*180/PI;
	double rght_ref_angle = atan(line.kin_inliers.at(line.kin_inliers.size() - 1).x/
								 line.kin_inliers.at(line.kin_inliers.size() - 1).y)*180/PI;
	Passage new_passage;
	new_passage.width += 1.8;

	if(left_ref_angle > -29)
	{
		new_passage.kin_middle.x = line.kin_inliers.at(0).x - 0.9*line.ldir_vec.kin.x;
		new_passage.kin_middle.y = line.kin_inliers.at(0).y - 0.9*line.ldir_vec.kin.y;
		this->passage.push_back(new_passage);
	}
/*
	if(rght_ref_angle < 29)
	{
		new_passage.kin_middle.x = line.kin_inliers.at(line.kin_inliers.size() - 1).x + 0.9*line.ldir_vec.kin.x;
		new_passage.kin_middle.y = line.kin_inliers.at(line.kin_inliers.size() - 1).y + 0.9*line.ldir_vec.kin.y;
		this->passage.push_back(new_passage);
	}
*/
};
