#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

static void help()
{
	cout << "this is a camera calibration sample." << endl
		<< "usage: calibration configurationfile" << endl
		<< "near the sample file you'll find the configuration file, which has detailed help of "
		"how to edit it.  it may be any opencv supported file format xml/yaml." << endl;
}
class settings
{
public:
	settings() : goodinput(false) {}
	enum pattern { not_existing, chessboard, circles_grid, asymmetric_circles_grid };
	enum inputtype { invalid, camera, video_file, image_list };

	void write(filestorage& fs) const                        //write serialization for this class
	{
		fs << "{" << "boardsize_width" << boardsize.width
			<< "boardsize_height" << boardsize.height
			<< "square_size" << squaresize
			<< "calibrate_pattern" << patterntouse
			<< "calibrate_nrofframetouse" << nrframes
			<< "calibrate_fixaspectratio" << aspectratio
			<< "calibrate_assumezerotangentialdistortion" << calibzerotangentdist
			<< "calibrate_fixprincipalpointatthecenter" << calibfixprincipalpoint

			<< "write_detectedfeaturepoints" << bwritepoints
			<< "write_extrinsicparameters" << bwriteextrinsics
			<< "write_outputfilename" << outputfilename

			<< "show_undistortedimage" << showundistorsed

			<< "input_fliparoundhorizontalaxis" << flipvertical
			<< "input_delay" << delay
			<< "input" << input
			<< "}";
	}
	void read(const filenode& node)                          //read serialization for this class
	{
		node["boardsize_width"] >> boardsize.width;
		node["boardsize_height"] >> boardsize.height;
		node["calibrate_pattern"] >> patterntouse;
		node["square_size"] >> squaresize;
		node["calibrate_nrofframetouse"] >> nrframes;
		node["calibrate_fixaspectratio"] >> aspectratio;
		node["write_detectedfeaturepoints"] >> bwritepoints;
		node["write_extrinsicparameters"] >> bwriteextrinsics;
		node["write_outputfilename"] >> outputfilename;
		node["calibrate_assumezerotangentialdistortion"] >> calibzerotangentdist;
		node["calibrate_fixprincipalpointatthecenter"] >> calibfixprincipalpoint;
		node["input_fliparoundhorizontalaxis"] >> flipvertical;
		node["show_undistortedimage"] >> showundistorsed;
		node["input"] >> input;
		node["input_delay"] >> delay;
		interprate();
	}
	void interprate()
	{
		goodinput = true;
		if (boardsize.width <= 0 || boardsize.height <= 0)
		{
			cerr << "invalid board size: " << boardsize.width << " " << boardsize.height << endl;
			goodinput = false;
		}
		if (squaresize <= 10e-6)
		{
			cerr << "invalid square size " << squaresize << endl;
			goodinput = false;
		}
		if (nrframes <= 0)
		{
			cerr << "invalid number of frames " << nrframes << endl;
			goodinput = false;
		}

		if (input.empty())      // check for valid input
			inputtype = invalid;
		else
		{
			if (input[0] >= '0' && input[0] <= '9')
			{
				stringstream ss(input);
				ss >> cameraid;
				inputtype = camera;
			}
			else
			{
				if (readstringlist(input, imagelist))
				{
					inputtype = image_list;
					nrframes = (nrframes < (int)imagelist.size()) ? nrframes : (int)imagelist.size();
				}
				else
					inputtype = video_file;
			}
			if (inputtype == camera)
				inputcapture.open(cameraid);
			if (inputtype == video_file)
				inputcapture.open(input);
			if (inputtype != image_list && !inputcapture.isopened())
				inputtype = invalid;
		}
		if (inputtype == invalid)
		{
			cerr << " inexistent input: " << input;
			goodinput = false;
		}

		flag = 0;
		if (calibfixprincipalpoint) flag |= cv_calib_fix_principal_point;
		if (calibzerotangentdist)   flag |= cv_calib_zero_tangent_dist;
		if (aspectratio)            flag |= cv_calib_fix_aspect_ratio;


		calibrationpattern = not_existing;
		if (!patterntouse.compare("chessboard")) calibrationpattern = chessboard;
		if (!patterntouse.compare("circles_grid")) calibrationpattern = circles_grid;
		if (!patterntouse.compare("asymmetric_circles_grid")) calibrationpattern = asymmetric_circles_grid;
		if (calibrationpattern == not_existing)
		{
			cerr << " inexistent camera calibration mode: " << patterntouse << endl;
			goodinput = false;
		}
		atimagelist = 0;

	}
	mat nextimage()
	{
		mat result;
		if (inputcapture.isopened())
		{
			mat view0;
			inputcapture >> view0;
			view0.copyto(result);
		}
		else if (atimagelist < (int)imagelist.size())
			result = imread(imagelist[atimagelist++], cv_load_image_color);

		return result;
	}

	static bool readstringlist(const string& filename, vector<string>& l)
	{
		l.clear();
		filestorage fs(filename, filestorage::read);
		if (!fs.isopened())
			return false;
		filenode n = fs.getfirsttoplevelnode();
		if (n.type() != filenode::seq)
			return false;
		filenodeiterator it = n.begin(), it_end = n.end();
		for (; it != it_end; ++it)
			l.push_back((string)*it);
		return true;
	}
public:
	size boardsize;            // the size of the board -> number of items by width and height
	pattern calibrationpattern;// one of the chessboard, circles, or asymmetric circle pattern
	float squaresize;          // the size of a square in your defined unit (point, millimeter,etc).
	int nrframes;              // the number of frames to use from the input for calibration
	float aspectratio;         // the aspect ratio
	int delay;                 // in case of a video input
	bool bwritepoints;         //  write detected feature points
	bool bwriteextrinsics;     // write extrinsic parameters
	bool calibzerotangentdist; // assume zero tangential distortion
	bool calibfixprincipalpoint;// fix the principal point at the center
	bool flipvertical;          // flip the captured images around the horizontal axis
	string outputfilename;      // the name of the file where to write
	bool showundistorsed;       // show undistorted images after calibration
	string input;               // the input ->



	int cameraid;
	vector<string> imagelist;
	int atimagelist;
	videocapture inputcapture;
	inputtype inputtype;
	bool goodinput;
	int flag;

private:
	string patterntouse;


};

static void read(const filenode& node, settings& x, const settings& default_value = settings())
{
	if (node.empty())
		x = default_value;
	else
		x.read(node);
}

enum { detection = 0, capturing = 1, calibrated = 2 };

bool runcalibrationandsave(settings& s, size imagesize, mat&  cameramatrix, mat& distcoeffs,
	vector<vector<point2f> > imagepoints);

int main(int argc, char* argv[])
{
	help();
	settings s;
	const string inputsettingsfile = argc > 1 ? argv[1] : "default.xml";
	filestorage fs(inputsettingsfile, filestorage::read); // read the settings
	if (!fs.isopened())
	{
		cout << "could not open the configuration file: \"" << inputsettingsfile << "\"" << endl;
		return -1;
	}
	fs["settings"] >> s;
	fs.release();                                         // close settings file

	if (!s.goodinput)
	{
		cout << "invalid input detected. application stopping. " << endl;
		return -1;
	}

	vector<vector<point2f> > imagepoints;
	mat cameramatrix, distcoeffs;
	size imagesize;
	int mode = s.inputtype == settings::image_list ? capturing : detection;
	clock_t prevtimestamp = 0;
	const scalar red(0, 0, 255), green(0, 255, 0);
	const char esc_key = 27;

	for (int i = 0;; ++i)
	{
		mat view;
		bool blinkoutput = false;

		view = s.nextimage();

		//-----  if no more image, or got enough, then stop calibration and show result -------------
		if (mode == capturing && imagepoints.size() >= (unsigned)s.nrframes)
		{
			if (runcalibrationandsave(s, imagesize, cameramatrix, distcoeffs, imagepoints))
				mode = calibrated;
			else
				mode = detection;
		}
		if (view.empty())          // if no more images then run calibration, save and stop loop.
		{
			if (imagepoints.size() > 0)
				runcalibrationandsave(s, imagesize, cameramatrix, distcoeffs, imagepoints);
			break;
		}


		imagesize = view.size();  // format input image.
		if (s.flipvertical)    flip(view, view, 0);

		vector<point2f> pointbuf;

		bool found;
		switch (s.calibrationpattern) // find feature points on the input format
		{
		case settings::chessboard:
			found = findchessboardcorners(view, s.boardsize, pointbuf,
				cv_calib_cb_adaptive_thresh | cv_calib_cb_fast_check | cv_calib_cb_normalize_image);
			break;
		case settings::circles_grid:
			found = findcirclesgrid(view, s.boardsize, pointbuf);
			break;
		case settings::asymmetric_circles_grid:
			found = findcirclesgrid(view, s.boardsize, pointbuf, calib_cb_asymmetric_grid);
			break;
		default:
			found = false;
			break;
		}

		if (found)                // if done with success,
		{
			// improve the found corners' coordinate accuracy for chessboard
			if (s.calibrationpattern == settings::chessboard)
			{
				mat viewgray;
				cvtcolor(view, viewgray, cv_bgr2gray);
				cornersubpix(viewgray, pointbuf, size(11, 11),
					size(-1, -1), termcriteria(cv_termcrit_eps + cv_termcrit_iter, 30, 0.1));
			}

			if (mode == capturing &&  // for camera only take new samples after delay time
				(!s.inputcapture.isopened() || clock() - prevtimestamp > s.delay*1e-3*clocks_per_sec))
			{
				imagepoints.push_back(pointbuf);
				prevtimestamp = clock();
				blinkoutput = s.inputcapture.isopened();
			}

			// draw the corners.
			drawchessboardcorners(view, s.boardsize, mat(pointbuf), found);
		}

		//----------------------------- output text ------------------------------------------------
		string msg = (mode == capturing) ? "100/100" :
			mode == calibrated ? "calibrated" : "press 'g' to start";
		int baseline = 0;
		size textsize = gettextsize(msg, 1, 1, 1, &baseline);
		point textorigin(view.cols - 2 * textsize.width - 10, view.rows - 2 * baseline - 10);

		if (mode == capturing)
		{
			if (s.showundistorsed)
				msg = format("%d/%d undist", (int)imagepoints.size(), s.nrframes);
			else
				msg = format("%d/%d", (int)imagepoints.size(), s.nrframes);
		}

		puttext(view, msg, textorigin, 1, 1, mode == calibrated ? green : red);

		if (blinkoutput)
			bitwise_not(view, view);

		//------------------------- video capture  output  undistorted ------------------------------
		if (mode == calibrated && s.showundistorsed)
		{
			mat temp = view.clone();
			undistort(temp, view, cameramatrix, distcoeffs);
		}

		//------------------------------ show image and check for input commands -------------------
		imshow("image view", view);
		char key = (char)waitkey(s.inputcapture.isopened() ? 50 : s.delay);

		if (key == esc_key)
			break;

		if (key == 'u' && mode == calibrated)
			s.showundistorsed = !s.showundistorsed;

		if (s.inputcapture.isopened() && key == 'g')
		{
			mode = capturing;
			imagepoints.clear();
		}
	}

	// -----------------------show the undistorted image for the image list ------------------------
	if (s.inputtype == settings::image_list && s.showundistorsed)
	{
		mat view, rview, map1, map2;
		initundistortrectifymap(cameramatrix, distcoeffs, mat(),
			getoptimalnewcameramatrix(cameramatrix, distcoeffs, imagesize, 1, imagesize, 0),
			imagesize, cv_16sc2, map1, map2);

		for (int i = 0; i < (int)s.imagelist.size(); i++)
		{
			view = imread(s.imagelist[i], 1);
			if (view.empty())
				continue;
			remap(view, rview, map1, map2, inter_linear);
			imshow("image view", rview);
			char c = (char)waitkey();
			if (c == esc_key || c == 'q' || c == 'q')
				break;
		}
	}


	return 0;
}

static double computereprojectionerrors(const vector<vector<point3f> >& objectpoints,
	const vector<vector<point2f> >& imagepoints,
	const vector<mat>& rvecs, const vector<mat>& tvecs,
	const mat& cameramatrix, const mat& distcoeffs,
	vector<float>& perviewerrors)
{
	vector<point2f> imagepoints2;
	int i, totalpoints = 0;
	double totalerr = 0, err;
	perviewerrors.resize(objectpoints.size());

	for (i = 0; i < (int)objectpoints.size(); ++i)
	{
		projectpoints(mat(objectpoints[i]), rvecs[i], tvecs[i], cameramatrix,
			distcoeffs, imagepoints2);
		err = norm(mat(imagepoints[i]), mat(imagepoints2), cv_l2);

		int n = (int)objectpoints[i].size();
		perviewerrors[i] = (float)std::sqrt(err*err / n);
		totalerr += err*err;
		totalpoints += n;
	}

	return std::sqrt(totalerr / totalpoints);
}

static void calcboardcornerpositions(size boardsize, float squaresize, vector<point3f>& corners,
	settings::pattern patterntype /*= settings::chessboard*/)
{
	corners.clear();

	switch (patterntype)
	{
	case settings::chessboard:
	case settings::circles_grid:
		for (int i = 0; i < boardsize.height; ++i)
			for (int j = 0; j < boardsize.width; ++j)
				corners.push_back(point3f(float(j*squaresize), float(i*squaresize), 0));
		break;

	case settings::asymmetric_circles_grid:
		for (int i = 0; i < boardsize.height; i++)
			for (int j = 0; j < boardsize.width; j++)
				corners.push_back(point3f(float((2 * j + i % 2)*squaresize), float(i*squaresize), 0));
		break;
	default:
		break;
	}
}

static bool runcalibration(settings& s, size& imagesize, mat& cameramatrix, mat& distcoeffs,
	vector<vector<point2f> > imagepoints, vector<mat>& rvecs, vector<mat>& tvecs,
	vector<float>& reprojerrs, double& totalavgerr)
{

	cameramatrix = mat::eye(3, 3, cv_64f);
	if (s.flag & cv_calib_fix_aspect_ratio)
		cameramatrix.at<double>(0, 0) = 1.0;

	distcoeffs = mat::zeros(8, 1, cv_64f);

	vector<vector<point3f> > objectpoints(1);
	calcboardcornerpositions(s.boardsize, s.squaresize, objectpoints[0], s.calibrationpattern);

	objectpoints.resize(imagepoints.size(), objectpoints[0]);

	//find intrinsic and extrinsic camera parameters
	double rms = calibratecamera(objectpoints, imagepoints, imagesize, cameramatrix,
		distcoeffs, rvecs, tvecs, s.flag | cv_calib_fix_k4 | cv_calib_fix_k5);

	cout << "re-projection error reported by calibratecamera: " << rms << endl;

	bool ok = checkrange(cameramatrix) && checkrange(distcoeffs);

	totalavgerr = computereprojectionerrors(objectpoints, imagepoints,
		rvecs, tvecs, cameramatrix, distcoeffs, reprojerrs);

	return ok;
}

// print camera parameters to the output file
static void savecameraparams(settings& s, size& imagesize, mat& cameramatrix, mat& distcoeffs,
	const vector<mat>& rvecs, const vector<mat>& tvecs,
	const vector<float>& reprojerrs, const vector<vector<point2f> >& imagepoints,
	double totalavgerr)
{
	filestorage fs(s.outputfilename, filestorage::write);

	time_t tm;
	time(&tm);
	struct tm t2;
	localtime_s(&t2, &tm);
	char buf[1024];
	strftime(buf, sizeof(buf) - 1, "%c", &t2);

	fs << "calibration_time" << buf;

	if (!rvecs.empty() || !reprojerrs.empty())
		fs << "nrofframes" << (int)std::max(rvecs.size(), reprojerrs.size());
	fs << "image_width" << imagesize.width;
	fs << "image_height" << imagesize.height;
	fs << "board_width" << s.boardsize.width;
	fs << "board_height" << s.boardsize.height;
	fs << "square_size" << s.squaresize;

	if (s.flag & cv_calib_fix_aspect_ratio)
		fs << "fixaspectratio" << s.aspectratio;

	if (s.flag)
	{
		sprintf_s(buf, "flags: %s%s%s%s",
			s.flag & cv_calib_use_intrinsic_guess ? " +use_intrinsic_guess" : "",
			s.flag & cv_calib_fix_aspect_ratio ? " +fix_aspectratio" : "",
			s.flag & cv_calib_fix_principal_point ? " +fix_principal_point" : "",
			s.flag & cv_calib_zero_tangent_dist ? " +zero_tangent_dist" : "");
		cvwritecomment(*fs, buf, 0);

	}

	fs << "flagvalue" << s.flag;

	fs << "camera_matrix" << cameramatrix;
	fs << "distortion_coefficients" << distcoeffs;

	fs << "avg_reprojection_error" << totalavgerr;
	if (!reprojerrs.empty())
		fs << "per_view_reprojection_errors" << mat(reprojerrs);

	if (!rvecs.empty() && !tvecs.empty())
	{
		cv_assert(rvecs[0].type() == tvecs[0].type());
		mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
		for (int i = 0; i < (int)rvecs.size(); i++)
		{
			mat r = bigmat(range(i, i + 1), range(0, 3));
			mat t = bigmat(range(i, i + 1), range(3, 6));

			cv_assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
			cv_assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
			//*.t() is matexpr (not mat) so we can use assignment operator
			r = rvecs[i].t();
			t = tvecs[i].t();
		}
		cvwritecomment(*fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0);
		fs << "extrinsic_parameters" << bigmat;
	}

	if (!imagepoints.empty())
	{
		mat imageptmat((int)imagepoints.size(), (int)imagepoints[0].size(), cv_32fc2);
		for (int i = 0; i < (int)imagepoints.size(); i++)
		{
			mat r = imageptmat.row(i).reshape(2, imageptmat.cols);
			mat imgpti(imagepoints[i]);
			imgpti.copyto(r);
		}
		fs << "image_points" << imageptmat;
	}
}

bool runcalibrationandsave(settings& s, size imagesize, mat&  cameramatrix, mat& distcoeffs, vector<vector<point2f> > imagepoints)
{
	vector<mat> rvecs, tvecs;
	vector<float> reprojerrs;
	double totalavgerr = 0;

	bool ok = runcalibration(s, imagesize, cameramatrix, distcoeffs, imagepoints, rvecs, tvecs,
		reprojerrs, totalavgerr);
	cout << (ok ? "calibration succeeded" : "calibration failed")
		<< ". avg re projection error = " << totalavgerr;

	if (ok)
		savecameraparams(s, imagesize, cameramatrix, distcoeffs, rvecs, tvecs, reprojerrs,
			imagepoints, totalavgerr);
	return ok;
}
