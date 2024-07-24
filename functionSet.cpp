#include"functionSet.h"

static std::vector<double> tcpOffset;

// getSensorForce补偿数据   原理见：https://www.yuque.com/lindong-9iuax/cs7vo1/hh2nkivsldpgwcxd
// 个人写的传感器那篇
bool isDataCompensation = false;
static Compensation_torque data;
static Compensation Fx_Compensation;
static Compensation Fy_Compensation;
static Compensation Fz_Compensation;
static Compensation Mx_Compensation;
static Compensation My_Compensation;
static Compensation Mz_Compensation;


//功能函数
QImage MatToQImage(const cv::Mat& mat)
{
	// 8-bits unsigned, NO. OF CHANNELS = 1
	if (mat.type() == CV_8UC1)
	{
		QImage image(mat.cols, mat.rows, QImage::Format_Indexed8);
		// Set the color table (used to translate colour indexes to qRgb values)
		image.setColorCount(256);
		for (int i = 0; i < 256; i++)
		{
			image.setColor(i, qRgb(i, i, i));
		}
		// Copy input Mat
		uchar* pSrc = mat.data;
		for (int row = 0; row < mat.rows; row++)
		{
			uchar* pDest = image.scanLine(row);
			memcpy(pDest, pSrc, mat.cols);
			pSrc += mat.step;
		}
		return image;
	}
	// 8-bits unsigned, NO. OF CHANNELS = 3
	else if (mat.type() == CV_8UC3)
	{
		// Copy input Mat
		const uchar* pSrc = mat.data;
		// Create QImage with same dimensions as input Mat
		QImage image(pSrc, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);
		return image.rgbSwapped();
	}
	else if (mat.type() == CV_8UC4)
	{
		// Copy input Mat
		const uchar* pSrc = mat.data;
		// Create QImage with same dimensions as input Mat
		QImage image(pSrc, mat.cols, mat.rows, mat.step, QImage::Format_ARGB32);
		return image.copy();
	}
	else
	{
		//出现错误
		return QImage();
	}
}

cv::Mat QImage2cvMat(QImage image)
{
	cv::Mat mat;
	switch (image.format())
	{
	case QImage::Format_ARGB32:
	case QImage::Format_RGB32:
	case QImage::Format_ARGB32_Premultiplied:
		mat = cv::Mat(image.height(), image.width(), CV_8UC4, (void*)image.bits(), image.bytesPerLine());
		break;
	case QImage::Format_RGB888:
		mat = cv::Mat(image.height(), image.width(), CV_8UC3, (void*)image.bits(), image.bytesPerLine());
		cv::cvtColor(mat, mat, cv::COLOR_BGR2RGB);
		break;
	case QImage::Format_Indexed8:
		mat = cv::Mat(image.height(), image.width(), CV_8UC1, (void*)image.bits(), image.bytesPerLine());
		break;
	}
	return mat;
}

void initTCPOffSet(ur_rtde::RTDEControlInterface* rtde_c) {

	tcpOffset = rtde_c->getTCPOffset();
	qDebug() << "tcpOffset :" << tcpOffset[0] << tcpOffset[1] <<
		tcpOffset[2] << tcpOffset[3] << tcpOffset[4] << tcpOffset[5] << endl;
}


void computeMotionCoordinateSystem(const Vec3d& sixth_axis,
	const Vec3d& end_effector_direction,
	const Vec3d& base_frame_z_axis,
	Vec3d& x_axis,
	Vec3d& y_axis,
	Vec3d& z_axis,
	ur_rtde::RTDEReceiveInterface* rtde_r) {

	// Z轴是第六轴
	z_axis = -sixth_axis;

	if (cv::norm(end_effector_direction) != 0) {
		// 计算X轴
		Vec3d temp = end_effector_direction.cross(base_frame_z_axis);
		x_axis = z_axis.cross(temp);
	}
	else {
		// 如果没有移动方向，移动坐标系为传感器坐标系绕x轴顺时针旋转180°
		auto pose = getSensorPose(rtde_r);
		cv::Mat base2sensor;
		cv::Rodrigues(cv::Vec3d(pose[3], pose[4], pose[5]), base2sensor);
		getAxisVecFromRotMat(x_axis, y_axis, z_axis,base2sensor);
		x_axis = x_axis;
		y_axis = -y_axis;
		z_axis = -z_axis;
		return;
	}
	//归一化
	normalize(x_axis, x_axis);
	normalize(z_axis, z_axis);

	// 计算Y轴
	y_axis = z_axis.cross(x_axis);
	normalize(y_axis, y_axis);
	return;
}

cv::Vec3d computeIntersectionLine(const cv::Vec3d& normal1, const cv::Vec3d& normal2) {
	// 计算叉积得到交线的方向向量
	cv::Vec3d intersectionLine = normal1.cross(normal2);

	// 归一化向量
	intersectionLine = intersectionLine / cv::norm(intersectionLine);

	return intersectionLine;
}


cv::Mat calculateRotationMatrix(const cv::Vec3d& axis1_1, const cv::Vec3d& axis1_2, const cv::Vec3d& axis1_3,
	const cv::Vec3d& axis2_1, const cv::Vec3d& axis2_2, const cv::Vec3d& axis2_3) {
	// 构建第一个坐标系的方向矩阵
	cv::Mat coord1 = (cv::Mat_<double>(3, 3) << axis1_1[0], axis1_2[0], axis1_3[0],
		axis1_1[1], axis1_2[1], axis1_3[1],
		axis1_1[2], axis1_2[2], axis1_3[2]);

	// 构建第二个坐标系的方向矩阵
	cv::Mat coord2 = (cv::Mat_<double>(3, 3) << axis2_1[0], axis2_2[0], axis2_3[0],
		axis2_1[1], axis2_2[1], axis2_3[1],
		axis2_1[2], axis2_2[2], axis2_3[2]);

	// 计算旋转矩阵：第一个坐标系矩阵的逆乘以第二个坐标系矩阵
	cv::Mat rotationMatrix = coord1.inv() * coord2;

	return rotationMatrix;
}

cv::Mat getRotationMatrixFromBase(const Vec3d& x_axis,
	const Vec3d& y_axis,
	const Vec3d& z_axis) {
	cv::Mat rotation_matrix = (cv::Mat_<double>(3, 3) << x_axis[0], y_axis[0], z_axis[0],
		x_axis[1], y_axis[1], z_axis[1],
		x_axis[2], y_axis[2], z_axis[2]);
	return rotation_matrix;
}

void getAxisVecFromRotMat(Vec3d& x_axis,
	Vec3d& y_axis,
	Vec3d& z_axis, const cv::Mat& rotation_matrix) {

	// 确保旋转矩阵是3x3的
	if (rotation_matrix.rows != 3 || rotation_matrix.cols != 3) {
		throw std::invalid_argument("Rotation matrix must be 3x3");
	}

	// 提取旋转矩阵的列，这些列代表B坐标系的轴在A坐标系中的表示
	x_axis = Vec3d(rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(2, 0));
	y_axis = Vec3d(rotation_matrix.at<double>(0, 1), rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(2, 1));
	z_axis = Vec3d(rotation_matrix.at<double>(0, 2), rotation_matrix.at<double>(1, 2), rotation_matrix.at<double>(2, 2));
}


std::vector<double> transformForceToNewCoordinateSystem(const std::vector<double>& force_in_base,
	const cv::Mat& rotation_matrix,
	const cv::Vec3d& origin_in_base) {


	std::vector<double> force_rotated = rotateForceToTarget(force_in_base, rotation_matrix);

	std::vector<double> force_transformed = TransformForceToNewFrame(force_rotated, origin_in_base);

	return force_transformed;
}



Vec3d getSixthAxisFromBasePose(std::vector<double> pose) {
	Vec3d kxita{ pose[3],pose[4],pose[5] };
	cv::Mat rotation_matrix;
	cv::Rodrigues(kxita, rotation_matrix);
	Vec3d baseUnitZ{ 0.0,0.0,1.0 };
	return cv::Mat(rotation_matrix * baseUnitZ);
}



std::vector<double> calculateToolPointSpeed(const std::vector<double>& tcp_speed,
	const Vec3d& tool_offset) {
	// 提取TCP的线速度和角速度
	Vec3d tcp_linear_speed(tcp_speed[0], tcp_speed[1], tcp_speed[2]);
	Vec3d tcp_angular_speed(tcp_speed[3], tcp_speed[4], tcp_speed[5]);

	// 将工具偏移量转换为Vec3d
	Vec3d tool_offset_vec(tool_offset[0], tool_offset[1], tool_offset[2]);

	// 通过外积计算由TCP的角速度引起的附加线速度
	Vec3d additional_speed = tcp_angular_speed.cross(tool_offset_vec);

	// 工具点的总速度是TCP的线速度加上附加线速度
	Vec3d tool_speed = tcp_linear_speed + additional_speed;

	// 工具点的角速度与TCP相同
	cv::Vec3d toolAngularSpeed = tcp_angular_speed;

	// 组合工具点的六维速度
	std::vector<double> toolSpeed = { tool_speed[0], tool_speed[1], tool_speed[2],
									   toolAngularSpeed[0], toolAngularSpeed[1], toolAngularSpeed[2] };

	return toolSpeed;
}



std::vector<double> getMotionCoordinateSystemForce(const Vec3d& sixth_axis,
	const Vec3d& end_effector_direction,
	const Vec3d& base_frame_z_axis,
	ur_rtde::RTDEReceiveInterface* rtde_r) {

	Vec3d x_axis;
	Vec3d y_axis;
	Vec3d z_axis;

	computeMotionCoordinateSystem(sixth_axis, end_effector_direction, base_frame_z_axis,
		x_axis, y_axis, z_axis,rtde_r);

	std::vector<double> sensorforce = getSensorForce(rtde_r,true,true);

	std::vector<double> sensorPose = getSensorPose(rtde_r);

	cv::Mat rotMat_base2motion = getRotationMatrixFromBase(x_axis, y_axis, z_axis);

	cv::Vec3d rotVec_base2sensor{ sensorPose[3],sensorPose[4],sensorPose[5] };

	cv::Mat rotMat_base2sensor;

	cv::Rodrigues(rotVec_base2sensor, rotMat_base2sensor);

	cv::Mat rotMat_sensor2motion = calculateRotationMatrix_BC(rotMat_base2sensor, rotMat_base2motion);

	cv::Vec3d rotVec_sensor2motion;

	cv::Rodrigues(rotMat_sensor2motion, rotVec_sensor2motion);

	return wrench_trans({ 0,0,0,rotVec_sensor2motion[0],rotVec_sensor2motion[1],rotVec_sensor2motion[2]
		}, sensorforce);

	//std::vector<double> senorforce = getSensorForce(rtde_r);
	//std::vector<double> pose = rtde_r->getTargetTCPPose();
	//std::vector<double> speed = rtde_r->getTargetTCPSpeed();

	//Vec3d tcp2sensor_senor{-tcpOffset[0],-tcpOffset[1],-tcpOffset[2] };

	//std::vector<double> sensorPose = getSensorPose(rtde_r);

	//Vec3d base2sensorRv{ sensorPose[3],sensorPose[4],sensorPose[5] };

	//cv::Mat base2sensorRm;

	//cv::Rodrigues(base2sensorRv, base2sensorRm);

	//Vec3d tcp2sensor_base = (cv::Mat)(base2sensorRm * tcp2sensor_senor);

	//Vec3d sensor_speed = calculateToolPointSpeed(speed, tcp2sensor_base);//传感器坐标系圆心的速度

	////基座标Z轴单位向量,在基座标系下的表示
	//Vec3d base_frame_z_axis = { 0,0,1 };

	//Vec3d unit_sisth_axis_dir = getSixthAxisFromBasePose(pose);

	//Vec3d end_effector_direction = { speed[0],speed[1],0 }; //不考虑z方向
	//cv::normalize(end_effector_direction, end_effector_direction);

	//if (cv::norm(end_effector_direction) != 0) {
	//	Vec3d motion_x_axis, motion_y_axis, motion_z_axis;
	//	computeMotionCoordinateSystem(unit_sisth_axis_dir, end_effector_direction,
	//		base_frame_z_axis, motion_x_axis, motion_y_axis, motion_z_axis);

	//	//基座标系到移动坐标系的旋转矩阵
	//	cv::Mat RotationMatrix_Base2Motion = getRotationMatrixFromBase(motion_x_axis, motion_y_axis, motion_z_axis);
	//	return rotateForceToTarget(force, RotationMatrix_Base2Motion);
	//}
	//else {
	//	return world_2tcp_force(force, pose);
	//}
}

cv::Mat getRotationMatrix(RotationAxis axis, double radians) {

	cv::Mat rotationMatrix(3, 3, CV_64F);
	switch (axis) {
	case RotationAxis::X_AXIS:
		rotationMatrix = (cv::Mat_<double>(3, 3) <<
			1, 0, 0,
			0, cos(radians), -sin(radians),
			0, sin(radians), cos(radians));
		break;
	case RotationAxis::Y_AXIS:
		rotationMatrix = (cv::Mat_<double>(3, 3) <<
			cos(radians), 0, sin(radians),
			0, 1, 0,
			-sin(radians), 0, cos(radians));
		break;
	case RotationAxis::Z_AXIS:
		rotationMatrix = (cv::Mat_<double>(3, 3) <<
			cos(radians), -sin(radians), 0,
			sin(radians), cos(radians), 0,
			0, 0, 1);
		break;
	}
	return rotationMatrix;
}


cv::Vec3d combineRotationVectors(const cv::Vec3d& rotVec1, const cv::Vec3d& rotVec2) {

	// 将旋转向量转换为旋转矩阵
	cv::Mat rotMat1, rotMat2;
	cv::Rodrigues(rotVec1, rotMat1);
	cv::Rodrigues(rotVec2, rotMat2);

	// 组合旋转矩阵
	cv::Mat combinedRotMat = rotMat1 * rotMat2;

	// 将组合后的旋转矩阵转换回旋转向量
	cv::Vec3d combinedRotVec;
	cv::Rodrigues(combinedRotMat, combinedRotVec);

	return combinedRotVec;
}


std::vector<double> rotateForceToTarget(const std::vector<double>& force, const cv::Mat& rotation_matrix) {
	// 确保力向量有6个元素
	if (force.size() != 6) {
		throw std::invalid_argument("The force vector must have 6 elements.");
	}

	// 确保旋转矩阵是3x3的
	if (rotation_matrix.rows != 3 || rotation_matrix.cols != 3) {
		throw std::invalid_argument("The rotation matrix must be 3x3.");
	}

	// 提取力和扭矩分量
	cv::Vec3d force_vector(force[0], force[1], force[2]);
	cv::Vec3d torque_vector(force[3], force[4], force[5]);

	cv::Mat rotation_matrix_inv = rotation_matrix.inv();

	// 将旋转矩阵应用到力和扭矩分量上
	cv::Vec3d rotated_force_vector = cv::Mat(rotation_matrix_inv * force_vector);
	cv::Vec3d rotated_torque_vector = cv::Mat(rotation_matrix_inv * torque_vector);

	// 创建一个新的六维力向量来存储旋转后的结果
	std::vector<double> rotated_force(6);

	// 存储旋转后的力分量
	rotated_force[0] = rotated_force_vector[0];
	rotated_force[1] = rotated_force_vector[1];
	rotated_force[2] = rotated_force_vector[2];

	// 存储旋转后的扭矩分量
	rotated_force[3] = rotated_torque_vector[0];
	rotated_force[4] = rotated_torque_vector[1];
	rotated_force[5] = rotated_torque_vector[2];

	return rotated_force;
}

cv::Vec3d rotateVecToTarget(const cv::Vec3d& Vec1, const cv::Mat& rotation_matrix) {
	
	// 确保旋转矩阵是3x3的
	if (rotation_matrix.rows != 3 || rotation_matrix.cols != 3) {
		throw std::invalid_argument("The rotation matrix must be 3x3.");
	}

	cv::Mat rotation_matrix_inv = rotation_matrix.inv();

	// 将旋转矩阵应用到向量上
	cv::Vec3d rotated_vector = cv::Mat(rotation_matrix_inv * Vec1);

	return rotated_vector;

}


std::vector<double> TransformForceToNewFrame(const std::vector<double>& force, const cv::Vec3d& distanceVector) {
	if (force.size() != 6) {
		throw std::invalid_argument("Force vector must have 6 elements.");
	}

	// 提取力和力矩
	cv::Vec3d forceVector(force[0], force[1], force[2]);
	cv::Vec3d torqueVector(force[3], force[4], force[5]);

	// 在新坐标系下计算力矩
	cv::Vec3d newTorque = torqueVector - distanceVector.cross(forceVector);

	qDebug() << "new :" << torqueVector[0] << torqueVector[1] << torqueVector[2] << '-' << distanceVector[0]
		<< distanceVector[1] << distanceVector[2] << 'X' << forceVector[0] << forceVector[1] << forceVector[2];

	// 返回新坐标系下的六维度力向量
	return { forceVector[0], forceVector[1], forceVector[2], newTorque[0], newTorque[1], newTorque[2] };
}



cv::Mat calculateRotationMatrix_BC(const cv::Mat& R_AB, const cv::Mat& R_AC) {
	
	// 检查输入矩阵的尺寸是否为3x3
	if (R_AB.rows != 3 || R_AB.cols != 3 || R_AC.rows != 3 || R_AC.cols != 3) {
		throw std::invalid_argument("Both rotation matrices must be 3x3.");
	}

	// 计算从B到A的旋转矩阵，即R_AB的转置
	cv::Mat R_BA = R_AB.t();

	// 计算从B到C的旋转矩阵，即R_BA乘以R_AC
	cv::Mat R_BC = R_BA * R_AC;

	return R_BC;
}


cv::Mat calculateRotationMatrix_Base2TCP(const std::vector<double>& pose) {
	
	Vec3d rxryrz{ pose[3],pose[4],pose[5] };
	cv::Mat RotationMatrix;
	cv::Rodrigues(rxryrz, RotationMatrix);
	return RotationMatrix;
}


std::vector<double> world_2tcp_force(const std::vector<double>& world_force, const std::vector<double>& in_pose)
{
	cv::Mat force = (cv::Mat_<double>(3, 1) << world_force[0], world_force[1], world_force[2]);//
	cv::Mat m_force = (cv::Mat_<double>(3, 1) << world_force[3], world_force[4], world_force[5]);// 3 * 1的力矩向量

	cv::Mat r_l = (cv::Mat_<double>(3, 1) << in_pose[3], in_pose[4], in_pose[5]);//旋转向量

	cv::Mat  R_M;
	cv::Rodrigues(r_l, R_M);

	cv::Mat tcp_force_xyz_mat = R_M * force;

	cv::Mat tcp_force_mxyz_mat = R_M * m_force;

	std::vector<double> tcp_force;
	tcp_force.push_back(tcp_force_xyz_mat.at<double>(0, 0));
	tcp_force.push_back(tcp_force_xyz_mat.at<double>(1, 0));
	tcp_force.push_back(tcp_force_xyz_mat.at<double>(2, 0));
	tcp_force.push_back(tcp_force_mxyz_mat.at<double>(0, 0));
	tcp_force.push_back(tcp_force_mxyz_mat.at<double>(1, 0));
	tcp_force.push_back(tcp_force_mxyz_mat.at<double>(2, 0));

	return tcp_force;
}



Vec3d get_Kxita_From_theta_x_and_theta_y_baseMotion(std::vector<double> pose, std::vector<double> speed
													, double theta_x, double theta_y, ur_rtde::RTDEReceiveInterface* rtde_r) {
	
	Vec3d x_axis, y_axis, z_axis;
	Vec3d sixth_axis = getSixthAxisFromBasePose(pose);
	Vec3d end_effector_direction{speed[0],speed[1],0};
	Vec3d base_frame_z_axis{0,0,1.0};

	computeMotionCoordinateSystem(sixth_axis, end_effector_direction, 
						base_frame_z_axis, x_axis, y_axis, z_axis,rtde_r);
	
	cv::Mat RotationMatrix = getRotationMatrixFromBase(x_axis, y_axis, z_axis);

	cv::Mat RotationMatrix_theta_x = getRotationMatrix(RotationAxis::X_AXIS, theta_x);
	
	cv::Mat RotationMatrix_theta_y = getRotationMatrix(RotationAxis::Y_AXIS, theta_y);

	RotationMatrix = RotationMatrix * RotationMatrix_theta_x * RotationMatrix_theta_y;

	Vec3d Kxita;

	cv::Rodrigues(RotationMatrix, Kxita);

	return Kxita;

}


cv::Vec3d CalculateRotationVector(const cv::Vec3d& Vec1, const cv::Vec3d& Vec2) {
	
	// 将向量标准化为单位向量
	cv::Vec3d v1 = Vec1 / cv::norm(Vec1);
	cv::Vec3d v2 = Vec2 / cv::norm(Vec2);

	// 计算旋转轴（两向量的叉积）
	cv::Vec3d axis = v1.cross(v2);
	double sin_angle = cv::norm(axis);
	double cos_angle = v1.dot(v2);

	// 如果两向量平行（叉积为零），返回零向量
	if (sin_angle < 1e-8) {
		return cv::Vec3d(0, 0, 0);
	}

	// 计算旋转角度(返回弧度)
	double angle = std::atan2(sin_angle, cos_angle);

	// 将旋转轴标准化并乘以角度，得到旋转向量（轴-角表示）
	return axis / sin_angle * angle;
}



cv::Vec3d convertRotaionMatrixToEulerAngles(const cv::Mat& rotationMatrix) {
	// 确保旋转矩阵是3x3矩阵
	CV_Assert(rotationMatrix.rows == 3 && rotationMatrix.cols == 3);

	double sy = sqrt(rotationMatrix.at<double>(0, 0) * rotationMatrix.at<double>(0, 0) +
		rotationMatrix.at<double>(1, 0) * rotationMatrix.at<double>(1, 0));

	bool singular = sy < 1e-6; // 检查奇异性

	double x, y, z;
	if (!singular) {
		x = atan2(rotationMatrix.at<double>(2, 1), rotationMatrix.at<double>(2, 2));
		y = atan2(-rotationMatrix.at<double>(2, 0), sy);
		z = atan2(rotationMatrix.at<double>(1, 0), rotationMatrix.at<double>(0, 0));
	}
	else {
		x = atan2(-rotationMatrix.at<double>(1, 2), rotationMatrix.at<double>(1, 1));
		y = atan2(-rotationMatrix.at<double>(2, 0), sy);
		z = 0;
	}
	return cv::Vec3d(x, y, z);
}


void ResizeVector(cv::Vec3d& vector, double newLength) {
	double originalLength = cv::norm(vector);

	if (originalLength < 1e-8) {
		vector = cv::Vec3d(0, 0, 0);
	}
	else {
		vector *= (newLength / originalLength);
	}
}

std::vector<double> getTCPForce(ur_rtde::RTDEReceiveInterface* rtde_r) {

	std::vector<double> ft = rtde_r->getActualTCPForce();
	
	std::vector<double> targetTCPPose = rtde_r->getTargetTCPPose();

	//qDebug() << "tcpOffset:" << tcpOffset[0] << tcpOffset[1] << tcpOffset[2] << tcpOffset[3] << tcpOffset[4] << tcpOffset[5] << endl;

	std::vector<double> poseInv = pose_inv(tcpOffset);

	std::vector<double> t_flange_in_base = poseTrans(targetTCPPose, poseInv);

	std::vector<double> tempPose{ 0,0,0,t_flange_in_base[3],t_flange_in_base[4],t_flange_in_base[5] };

	std::vector<double> flange_rot = pose_inv(tempPose);

	tempPose = { ft[0],ft[1],ft[2],0,0,0 };

	std::vector<double> f = poseTrans(flange_rot, tempPose);

	tempPose = { ft[3],ft[4],ft[5],0,0,0 };

	std::vector<double> t = poseTrans(flange_rot, tempPose);

	std::vector<double> wrench_at_tool_flange{ f[0],f[1],f[2],t[0],t[1],t[2] };

	//qDebug() << "wrench_at_tool_flange:" << wrench_at_tool_flange[0] << wrench_at_tool_flange[1] << wrench_at_tool_flange[2] << wrench_at_tool_flange[3] << wrench_at_tool_flange[4] << wrench_at_tool_flange[5] << endl;
	auto temp = wrench_trans(tcpOffset, wrench_at_tool_flange);
	//qDebug() << "temp:" << temp[0] << temp[1] << temp[2] << temp[3] << temp[4] << wrench_at_tool_flange[5] << endl;

	return temp;

	//def get_tcp_force_tool() :
	//	force_torque = get_tcp_force()
	//	force_B = p[force_torque[0], force_torque[1], force_torque[2], 0, 0, 0]
	//	torque_B = p[force_torque[3], force_torque[4], force_torque[5], 0, 0, 0]
	//	tcp = get_actual_tcp_pose()
	//	rotation_BT = p[0, 0, 0, tcp[3], tcp[4], tcp[5]]
	//	force_T = pose_trans(pose_inv(rotation_BT), force_B)
	//	torque_T = pose_trans(pose_inv(rotation_BT), torque_B)
	//	force_torque_T = p[force_T[0], force_T[1], force_T[2], torque_T[0], torque_T[1], torque_T[2]]
	//	return force_torque_T
	//end

	//std::vector<double> force_torque = rtde_r->getActualTCPForce();

	//std::vector<double> force_B{ force_torque[0], force_torque[1], force_torque[2], 0, 0, 0 };

	//std::vector<double> torque_B{ force_torque[3], force_torque[4], force_torque[5], 0, 0, 0 };

	//std::vector<double> tcp = rtde_r->getActualTCPPose();

	//std::vector<double> rotation_BT{ 0, 0, 0, tcp[3], tcp[4], tcp[5] };

	//std::vector<double> force_T = poseTrans(pose_inv(rotation_BT), force_B);

	//std::vector<double> torque_T = poseTrans(pose_inv(rotation_BT), torque_B);

	//return { force_T[0], force_T[1], force_T[2], torque_T[0], torque_T[1], torque_T[2] };
}

std::vector<double> getSensorForce(ur_rtde::RTDEReceiveInterface* rtde_r,bool isCompensate, bool enableFilter) {

	std::vector<double> ft = rtde_r->getActualTCPForce();

	//-------------------------------------滤波增加部分(一阶低通滤波)--------------------------------------

	// 根据是否启用滤波选择处理方式
	std::vector<double> filteredForce;

	if (enableFilter) {
		filteredForce = lowPassFilter(ft, 2 * M_PI * 1e-3);  // 根据需要调整alpha值
	}
	else {
		filteredForce = ft;
	}

	//------------------------------------------------------------------------------------------------

	std::vector<double> targetTCPPose = rtde_r->getTargetTCPPose();

	//qDebug() << "tcpOffset:" << tcpOffset[0] << tcpOffset[1] << tcpOffset[2] << tcpOffset[3] << tcpOffset[4] << tcpOffset[5] << endl;

	std::vector<double> poseInv = pose_inv(tcpOffset);

	std::vector<double> t_flange_in_base = poseTrans(targetTCPPose, poseInv);

	std::vector<double> tempPose{ 0,0,0,t_flange_in_base[3],t_flange_in_base[4],t_flange_in_base[5] };

	std::vector<double> flange_rot = pose_inv(tempPose);

	tempPose = { filteredForce[0],filteredForce[1],filteredForce[2],0,0,0 };

	std::vector<double> f = poseTrans(flange_rot, tempPose);

	tempPose = { filteredForce[3],filteredForce[4],filteredForce[5],0,0,0 };

	std::vector<double> t = poseTrans(flange_rot, tempPose);

	std::vector<double> wrench_at_tool_flange{ f[0],f[1],f[2],t[0],t[1],t[2] };


	//-------------------------------------------------应用卡尔曼滤波--------------------------------------------------
	
	static KalmanFilter filters[6] = {
		{0.1, 0.1}, {0.1, 0.1}, {0.1, 0.1}, {0.1, 0.1}, {0.1, 0.1}, {0.1, 0.1}
	};
	if (enableFilter) {
		for (int i = 0; i < 6; ++i) {
			wrench_at_tool_flange[i] = filters[i].update(wrench_at_tool_flange[i]);
		}
	}
	
	//---------------------------------------------------------------------------------------------------------------

	//-----------------------------------------------开启周期误差补偿---------------------------------------------------

	if (isCompensate && isDataCompensation) {

		auto caculate = [&](double theta, const std::vector<double>& frequencies, const std::vector<double>& a_k,
			const std::vector<double>& b_k)->double {

				double signal = 0.0;
				for (size_t i = 0; i < frequencies.size(); ++i) {
					signal += a_k[i] * std::cos(2 * M_PI * frequencies[i] * theta)
						+ b_k[i] * std::sin(2 * M_PI * frequencies[i] * theta);
				}
				return signal;
		};

		std::vector<double> Axis_deg = rtde_r->getTargetQ();

		double theta = Axis_deg[5];

		//补偿掉周期误差

		wrench_at_tool_flange[0] -= caculate(theta, Fx_Compensation.frequencies, Fx_Compensation.a_k, Fx_Compensation.b_k);

		wrench_at_tool_flange[1] -= caculate(theta, Fy_Compensation.frequencies, Fy_Compensation.a_k, Fy_Compensation.b_k);

		wrench_at_tool_flange[2] -= caculate(theta, Fz_Compensation.frequencies, Fz_Compensation.a_k, Fz_Compensation.b_k);

		wrench_at_tool_flange[3] -= caculate(theta, Mx_Compensation.frequencies, Mx_Compensation.a_k, Mx_Compensation.b_k);

		wrench_at_tool_flange[4] -= caculate(theta, My_Compensation.frequencies, My_Compensation.a_k, My_Compensation.b_k);

		//以下补偿对应 语雀传感器部分 --以前的猜想

		//wrench_at_tool_flange[3] -= data.M_e_x5 * cos(theta) + data.M_e_y5 * sin(theta) + data.M_x0;

		//wrench_at_tool_flange[4] -= -data.M_e_x5 * sin(theta) + data.M_e_y5 * cos(theta) + data.M_y0;

		//wrench_at_tool_flange[5] -= data.M_e_z5 + data.M_z0;

	}

	//----------------------------------------------------------------------------------------------------------------------

	return wrench_at_tool_flange;
}


std::vector<double> getSensorPose(ur_rtde::RTDEReceiveInterface* rtde_r) {

	std::vector<double> targetTCPPose = rtde_r->getTargetTCPPose();

	//qDebug() << "tcpOffset:" << tcpOffset[0] << tcpOffset[1] << tcpOffset[2] << tcpOffset[3] << tcpOffset[4] << tcpOffset[5] << endl;

	std::vector<double> poseInv = pose_inv(tcpOffset);

	std::vector<double> t_flange_in_base = poseTrans(targetTCPPose, poseInv);

	return t_flange_in_base;
}

std::vector<double> getSensorSpeed(ur_rtde::RTDEReceiveInterface* rtde_r) {

	Vec3d Vec_tcp2sensor{ -tcpOffset[0],-tcpOffset[1],-tcpOffset[2] };

	auto sensorPose = getSensorPose(rtde_r);

	cv::Vec3d Vec_base2sensor{ sensorPose[3],sensorPose[4],sensorPose[5] };

	cv::Mat Mat_base2sensor;

	cv::Rodrigues(Vec_base2sensor , Mat_base2sensor);

	cv::Vec3d tcpOffset_base = (cv::Mat)(Mat_base2sensor * Vec_tcp2sensor);

	return calculateToolPointSpeed(rtde_r->getTargetTCPSpeed(), tcpOffset_base);
}



std::pair<std::vector<double>, cv::Mat> getMyFormulaCDSForceAndBase2Formula( ur_rtde::RTDEReceiveInterface* rtde_r) {

	std::vector<double> ft = rtde_r->getActualTCPForce();

	std::vector<double> targetTCPPose = rtde_r->getTargetTCPPose();

	//qDebug() << "tcpOffset:" << tcpOffset[0] << tcpOffset[1] << tcpOffset[2] << tcpOffset[3] << tcpOffset[4] << tcpOffset[5] << endl;

	std::vector<double> poseInv = pose_inv(tcpOffset);

	std::vector<double> t_flange_in_base = poseTrans(targetTCPPose, poseInv);

	std::vector<double> tempPose{ 0,0,0,t_flange_in_base[3],t_flange_in_base[4],t_flange_in_base[5] };

	std::vector<double> flange_rot = pose_inv(tempPose);

	tempPose = { ft[0],ft[1],ft[2],0,0,0 };

	std::vector<double> f = poseTrans(flange_rot, tempPose);

	tempPose = { ft[3],ft[4],ft[5],0,0,0 };

	std::vector<double> t = poseTrans(flange_rot, tempPose);

	std::vector<double> wrench_at_tool_flange{ f[0],f[1],f[2],t[0],t[1],t[2] };

	//右乘相对于当前坐标系,为了符合公式，要对Y轴旋转180度
	auto Y_pi = getRotationMatrix(RotationAxis::Y_AXIS, M_PI);

	std::vector<double> myFormulaCDSForce = rotateForceToTarget(wrench_at_tool_flange, Y_pi);

	cv::Vec3d RotationVector{ t_flange_in_base[3],t_flange_in_base[4],t_flange_in_base[5] };
	cv::Mat RotationMatrix; //这是基座标系到法兰盘的旋转矩阵
	cv::Rodrigues(RotationVector, RotationMatrix);

	cv::Mat Base2Formula = RotationMatrix * Y_pi; //右乘相对于当前坐标系,为了符合公式，要对Y轴旋转180度

	return { myFormulaCDSForce , Base2Formula };

}


std::vector<double> getNewPoseFromCurPoseAndRotateVec(const std::vector<double>& pose, const cv::Vec3d& RotateVec) {

	cv::Vec3d curRotationVec{ pose[3],pose[4],pose[5] };

	cv::Mat curRotationMatrix;

	cv::Rodrigues(curRotationVec, curRotationMatrix);

	cv::Mat RotationMatrix;

	cv::Rodrigues(RotateVec, RotationMatrix);

	curRotationMatrix =  RotationMatrix * curRotationMatrix; //因为是旋转矢量是基于世界坐标系的，所以是相对于固定坐标系旋转 那就是左乘

	cv::Rodrigues(curRotationMatrix, curRotationVec);

	return { pose[0],pose[1],pose[2],curRotationVec[0],curRotationVec[1],curRotationVec[2] };
}


cv::Vec3d averageRotationVector(const std::vector<cv::Vec3d>& rotation_vectors) {
	// 转换旋转向量为旋转矩阵
	std::vector<cv::Mat> rotation_matrices;
	for (const auto& vec : rotation_vectors) {
		cv::Mat rotation_matrix;
		cv::Rodrigues(vec, rotation_matrix);
		rotation_matrices.push_back(rotation_matrix);
	}

	// 计算所有旋转矩阵的平均值
	cv::Mat sum = cv::Mat::zeros(3, 3, CV_64F);
	for (const auto& mat : rotation_matrices) {
		sum += mat;
	}
	sum /= static_cast<double>(rotation_vectors.size());

	// 使用SVD找到最接近的正交矩阵
	cv::Mat U, W, Vt;
	cv::SVD::compute(sum, W, U, Vt);
	cv::Mat average_matrix = U * Vt;

	// 将平均旋转矩阵转换回旋转向量
	cv::Vec3d average_rotation_vector;
	cv::Rodrigues(average_matrix, average_rotation_vector);

	return average_rotation_vector;
}


double calculateD(const std::vector<double>& forceA, const std::vector<double>& forceB, double x, double k) {
	// 确保输入向量长度正确
	if (forceA.size() < 6 || forceB.size() < 6) {
		throw std::invalid_argument("Input vectors must have at least 6 elements.");
	}

	// 提取力矩分量
	double tauAx = forceA[3];
	double tauAy = forceA[4];
	double tauBx = forceB[3];
	double tauBy = forceB[4];

	// 提取力分量
	double Fx = forceA[0];
	double Fy = forceA[1];

	// 计算平移距离d
	double d = 0.0;
	if (Fy != 0) {
		d = x - (tauBx - tauAx) / Fy;
	}
	else if (Fx != 0) {
		d = x + k + (tauBy - tauAy) / Fx;
	}
	else {
		throw std::runtime_error("Cannot calculate displacement: both Fx and Fy are zero.");
	}

	return d;
}


std::vector<double> pose_inv(const std::vector<double>& pose) {

	// 检查输入向量的大小
	if (pose.size() != 6) {
		throw std::invalid_argument("Invalid pose size");
	}

	// 分离平移向量和旋转向量
	std::vector<double> t(pose.begin(), pose.begin() + 3);
	std::vector<double> r(pose.begin() + 3, pose.end());

	// 将旋转向量转换为旋转矩阵
	cv::Mat R;
	cv::Rodrigues(cv::Mat(r), R);
	// 计算旋转矩阵的逆（转置）
	cv::Mat R_inv = R.t();
	// 将逆旋转矩阵转换回旋转向量
	std::vector<double> r_inv;
	cv::Rodrigues(R_inv, r_inv);

	// 逆平移向量需要考虑旋转
	cv::Mat t_mat(t); // 将平移向量转换为 Mat
	cv::Mat t_inv_mat = -R_inv * t_mat;
	std::vector<double> t_inv = { t_inv_mat.at<double>(0), t_inv_mat.at<double>(1), t_inv_mat.at<double>(2) };

	// 组合逆平移向量和逆旋转向量
	std::vector<double> inv_pose;
	inv_pose.insert(inv_pose.end(), t_inv.begin(), t_inv.end());
	inv_pose.insert(inv_pose.end(), r_inv.begin(), r_inv.end());

	return inv_pose;
}


std::vector<double> poseTrans(const std::vector<double>& p_from, const std::vector<double>& p_from_to) {
	// Ensure the input vectors have the correct size (translation + rotation)
	if (p_from.size() != 6 || p_from_to.size() != 6) {
		throw std::invalid_argument("Input vectors must have a size of 6");
	}

	// Convert the input vectors to transformation matrices
	cv::Mat T_from = cv::Mat::eye(4, 4, CV_64F); // Initialize as 4x4 identity matrix
	cv::Mat T_from_to = cv::Mat::eye(4, 4, CV_64F);

	// Assuming the first three elements are translation and the next three are rotation (Euler angles)
	T_from.at<double>(0, 3) = p_from[0]; // Translation x
	T_from.at<double>(1, 3) = p_from[1]; // Translation y
	T_from.at<double>(2, 3) = p_from[2]; // Translation z

	T_from_to.at<double>(0, 3) = p_from_to[0]; // Translation x
	T_from_to.at<double>(1, 3) = p_from_to[1]; // Translation y
	T_from_to.at<double>(2, 3) = p_from_to[2]; // Translation z

	// Apply rotation to the matrices
	cv::Mat R_from, R_from_to;
	cv::Rodrigues(cv::Vec3d(p_from[3], p_from[4], p_from[5]), R_from); // Convert rotation vector to rotation matrix
	cv::Rodrigues(cv::Vec3d(p_from_to[3], p_from_to[4], p_from_to[5]), R_from_to);

	R_from.copyTo(T_from(cv::Rect(0, 0, 3, 3)));
	R_from_to.copyTo(T_from_to(cv::Rect(0, 0, 3, 3)));

	// Compute the resulting transformation matrix
	cv::Mat T_result = T_from * T_from_to;

	// Convert the resulting transformation matrix back to a spatial vector
	std::vector<double> result(6);
	cv::Vec3d rot_vec;
	cv::Rodrigues(T_result(cv::Rect(0, 0, 3, 3)), rot_vec); // Convert rotation matrix to rotation vector

	result[0] = T_result.at<double>(0, 3); // Translation x
	result[1] = T_result.at<double>(1, 3); // Translation y
	result[2] = T_result.at<double>(2, 3); // Translation z
	result[3] = rot_vec[0]; // Rotation around x
	result[4] = rot_vec[1]; // Rotation around y
	result[5] = rot_vec[2]; // Rotation around z

	return result;
}



std::vector<double> wrench_trans(const std::vector<double>& T_from_to, const std::vector<double>& w_from) {
	// 验证输入
	if (T_from_to.size() != 6 || w_from.size() != 6) {
		throw std::invalid_argument("Invalid input dimensions.");
	}

	// 提取旋转向量和平移向量
	cv::Mat rvec(3, 1, CV_64F);
	for (int i = 0; i < 3; ++i) {
		rvec.at<double>(i) = T_from_to[i + 3];
	}

	cv::Mat tvec(3, 1, CV_64F);
	for (int i = 0; i < 3; ++i) {
		tvec.at<double>(i) = T_from_to[i];
	}

	// 将旋转向量转换为旋转矩阵
	cv::Mat R;
	cv::Rodrigues(rvec, R);

	//// 构建4x4齐次变换矩阵
	//cv::Mat T(4, 4, R.type(), cv::Scalar::all(0));
	//R.copyTo(T(cv::Rect(0, 0, 3, 3)));
	//tvec.copyTo(T(cv::Rect(3, 0, 1, 3)));
	//T.at<double>(3, 3) = 1.0;

	// 提取力和力矩
	cv::Mat F(3, 1, CV_64F), M(3, 1, CV_64F);
	for (int i = 0; i < 3; ++i) {
		F.at<double>(i) = w_from[i];
		M.at<double>(i) = w_from[i + 3];
	}

	R = R.inv();

	// 变换力和力矩
	cv::Mat F_to = R * F;
	cv::Mat M_to = R * M - tvec.cross(F_to);
	
	
	cv::Mat b = R * M;
	cv::Vec3d a = tvec.cross(F_to);
	
	//qDebug() <<b.at<double>(0,0) << b.at<double>(1, 0) << b.at<double>(2, 0) << '/' << a[0] << a[1] << a[2];

	// 转换回向量形式
	std::vector<double> w_to(6);
	for (int i = 0; i < 3; ++i) {
		w_to[i] = F_to.at<double>(i);
		w_to[i + 3] = M_to.at<double>(i);
	}

	return w_to;
}

void change_Compensation_torque(const double& M_e_x5, const double& M_e_y5, const double& M_e_z5,
	const double& M_x0, const double& M_y0, const double& M_z0,bool isCompensation) {

	data.M_e_x5 = M_e_x5;
	data.M_e_y5 = M_e_y5;
	data.M_e_z5 = M_e_z5;
	data.M_x0 = M_x0;
	data.M_y0 = M_y0;
	data.M_z0 = M_z0;

	isDataCompensation = isCompensation;

	return;
}

void change_Compensation(const std::pair<std::vector<double>, std::vector<double>>& P_Fx_ak_bk,
	const std::pair<std::vector<double>, std::vector<double>>& P_Fy_ak_bk,
	const std::pair<std::vector<double>, std::vector<double>>& P_Fz_ak_bk,
	const std::pair<std::vector<double>, std::vector<double>>& P_Mx_ak_bk,
	const std::pair<std::vector<double>, std::vector<double>>& P_My_ak_bk,
	const std::vector<double>& Fx_frequencies,
	const std::vector<double>& Fy_frequencies,
	const std::vector<double>& Fz_frequencies,
	const std::vector<double>& Mx_frequencies,
	const std::vector<double>& My_frequencies,
	bool isCompensation) {

	isDataCompensation = isCompensation;

	Fx_Compensation.a_k = P_Fx_ak_bk.first;
	Fx_Compensation.b_k = P_Fx_ak_bk.second;
	Fx_Compensation.frequencies = Fx_frequencies;

	Fy_Compensation.a_k = P_Fy_ak_bk.first;
	Fy_Compensation.b_k = P_Fy_ak_bk.second;
	Fy_Compensation.frequencies = Fy_frequencies;

	Fz_Compensation.a_k = P_Fz_ak_bk.first;
	Fz_Compensation.b_k = P_Fz_ak_bk.second;
	Fz_Compensation.frequencies = Fz_frequencies;

	Mx_Compensation.a_k = P_Mx_ak_bk.first;
	Mx_Compensation.b_k = P_Mx_ak_bk.second;
	Mx_Compensation.frequencies = Mx_frequencies;

	My_Compensation.a_k = P_My_ak_bk.first;
	My_Compensation.b_k = P_My_ak_bk.second;
	My_Compensation.frequencies = My_frequencies;

}



std::pair<std::vector<double>, std::vector<double>> fitCosineSineComponents(
	const std::vector<double>& frequencies,
	const std::vector<double>& anglesInRadians,
	const std::vector<double>& observedValues)
{

	int numDataPoints = anglesInRadians.size();
	int numFrequencies = frequencies.size();

	// 检查观测值数组和角度数组的大小是否一致
	if (observedValues.size() != numDataPoints) {
		std::cerr << "Error: The size of observed values does not match the number of angles." << std::endl;
		return {}; // 返回空的结果
	}

	// 创建设计矩阵
	cv::Mat A(numDataPoints, 2 * numFrequencies, CV_64F);
	for (int i = 0; i < numDataPoints; ++i) {
		for (int j = 0; j < numFrequencies; ++j) {
			A.at<double>(i, 2 * j) = std::cos(2 * CV_PI * frequencies[j] * anglesInRadians[i]);
			A.at<double>(i, 2 * j + 1) = std::sin(2 * CV_PI * frequencies[j] * anglesInRadians[i]);
		}
	}

	// 将观测值转换为OpenCV的Mat类型
	cv::Mat b(numDataPoints, 1, CV_64F);
	for (int i = 0; i < numDataPoints; ++i) {
		b.at<double>(i) = observedValues[i];
	}

	// 解最小二乘问题
	cv::Mat x;
	cv::solve(A, b, x, cv::DECOMP_SVD);

	// 提取系数a_k和b_k
	std::vector<double> a_k;
	std::vector<double> b_k;
	for (int j = 0; j < numFrequencies; ++j) {
		a_k.push_back(x.at<double>(2 * j));
		b_k.push_back(x.at<double>(2 * j + 1));
	}

	return { a_k, b_k };
}



cv::Mat solveLeastSquaresNormalEquation(const cv::Mat& A, const cv::Mat& b) {
	// 计算A的转置
	cv::Mat At = A.t();

	// 计算AtA和Atb
	cv::Mat AtA = At * A;
	cv::Mat Atb = At * b;

	// 计算解向量x = (AtA)^{-1} Atb
	cv::Mat x = AtA.inv(cv::DECOMP_SVD) * Atb;

	return x;
}


KalmanFilter::KalmanFilter(double process_noise, double measurement_noise)
	: estimate(0.0),
	error_covariance(1.0),
	process_noise(process_noise),
	measurement_noise(measurement_noise),
	kalman_gain(0.0) {}

double KalmanFilter::update(double measurement) {
	// 预测更新
	kalman_gain = error_covariance / (error_covariance + measurement_noise);
	estimate = estimate + kalman_gain * (measurement - estimate);
	error_covariance = (1 - kalman_gain) * error_covariance + process_noise;
	return estimate;
}


std::vector<double> lowPassFilter(const std::vector<double>& current,double alpha)
{
	static std::vector<double> previous; // 静态变量用于保存上一次的滤波结果
	if (previous.empty()) {
		previous = current; // 初始化时用当前读数赋值
	}
	std::vector<double> filtered(current.size());
	for (size_t i = 0; i < current.size(); ++i) {
		filtered[i] = alpha * current[i] + (1 - alpha) * previous[i];
	}
	previous = filtered; // 更新上一次的力值
	return filtered;
}


double Cal_EulerKFilter(
	const double& MeasureData,
	std::list<double>& MeasureDatas,
	const int& idv_FillterDelay,
	const double& idv_MNoiseCov,
	const double& idv_PNoiseCov)
{
	double MeasureDataByFilter = 0;
	MeasureDatas.push_back(MeasureData);

	//绕Y旋转的角速度K滤波
	if (MeasureDatas.size() < idv_FillterDelay)
	{
		MeasureDataByFilter = MeasureData;
	}
	else
	{
		MeasureDatas.pop_front();
		std::vector<double> vec;
		vec.assign(MeasureDatas.begin(), MeasureDatas.end());
		MeasureDataByFilter = Cal_KalmanFilter(vec, idv_MNoiseCov, idv_PNoiseCov);
	}
	return MeasureDataByFilter;
}

double Cal_KalmanFilter(std::vector<double> ilv_MeasureDatas,
	double idv_MNoiseCov,
	double idv_PNoiseCov)
{
	//估计值
	std::vector<double> ldv_EstimationDatas = ilv_MeasureDatas;

	//增益
	double ldv_Gain = 0;

	//估计值初值
	ldv_EstimationDatas[0] = 1;

	//过程初值
	double ldv_ProcessData = 10;

	int temp;//防报错用的 只是用来 赋值为i - 1，如下
	for (int i = 1; i < ilv_MeasureDatas.size(); i++)
	{
		temp = i - 1;
		ldv_EstimationDatas[i] = ldv_EstimationDatas[temp];

		ldv_ProcessData = ldv_ProcessData + idv_PNoiseCov;
		ldv_Gain = ldv_ProcessData / (ldv_ProcessData + idv_MNoiseCov);

		ldv_EstimationDatas[i]
			= ldv_EstimationDatas[i]
			+ ldv_Gain * (ilv_MeasureDatas[i] - ldv_EstimationDatas[i]);
		ldv_ProcessData = (1 - ldv_Gain) * ldv_ProcessData;
	}
	return ldv_EstimationDatas[ldv_EstimationDatas.size() - 1];
}



bool saveToCsv(const std::vector<std::vector<double>>& data, const std::string& path) {
	
	std::ofstream outFile(path);

	// 检查文件是否成功打开
	if (!outFile.is_open()) {
		return false;
	}

	// 遍历数据，将其写入文件
	for (const auto& row : data) {
		for (size_t i = 0; i < row.size(); ++i) {
			outFile << row[i];
			if (i < row.size() - 1) {
				outFile << ","; // 在元素之间添加逗号分隔符
			}
		}
		outFile << "\n"; // 每行结束后添加换行符
	}

	outFile.close();
	return true;
}



std::vector<std::vector<double>> readFromCsv(const std::string& path) {
	std::vector<std::vector<double>> data;
	std::ifstream inFile(path);

	// 检查文件是否成功打开
	if (!inFile.is_open()) {
		throw std::runtime_error("Cannot open file: " + path);
	}

	std::string line;
	// 逐行读取数据
	while (std::getline(inFile, line)) {

		std::vector<double> row;
		std::stringstream ss(line);
		std::string cell;

		// 逐个读取当前行的每个值
		while (std::getline(ss, cell, ',')) {
			// 将读取的字符串转换为double，并添加到当前行
			row.push_back(std::stod(cell));
		}

		// 将当前行添加到最终数据中
		data.push_back(row);
	}

	inFile.close();
	return data;
}