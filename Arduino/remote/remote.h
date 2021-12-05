/**
 * \file remote.h
 * \brief Definitions for the remote sketch
 */

#ifndef REMOTE_H_
#define REMOTE_H_


typedef enum _WfmDeemphasisMode {
	DeemphasisNone = 0,
	Deemphasis50us = 1,
	Deemphasis75us = 2,
	DeemphasisDefault = 0
} wfm_mode_t;

typedef enum _NoiseBlankerMode {
	NoiseBlankerOff = 0,
	NoiseBlankerBwWide = 1,
	NoiseBlankerBwNarrow = 2,
	NoiseBlankerDefault = 0
} nb_mode_t;


typedef enum _AgcMode {
	AGCModeOff = 0,
	AGCModeSlow = 1,
	AGCModeMedium = 2,
	AGCModeFast = 3,
	AGCModeDefault = 0
} agc_mode_t;


/**
 * \brief Band Definition
 */
typedef struct {
	String band_name;		/*!< String Name representing band */
	uint32_t band_lower;	/*!< Band lower edge, in Hz */
	uint32_t band_mid;		/*!< Band center, in Hz */
	uint32_t band_upper;	/*!< Band upper edge, in Hz */
} band_type;


/*! @todo: check middle freq! */
const band_type bands[BAND_DEFINITIONS_QTY] = {
	{
		"2200m",
		135700,
		136750,
		137800
	}, {
		"630m",
		472000,
		475500,
		479000
	}, {
		"160m",
		1800000,
		1900000,
		2000000
	}, {
		"80m",
		3500000,
		3750000,
		4000000
	}, {
		"60m",
		5330500,
		5368500,
		5406500
	}, {
		"40m",
		7000000,
		7150000,
		7300000
	}, {
		"30m",
		10100000,
		10125000,
		10150000
	}, {
		"20m",
		14000000,
		14175000,
		14350000
	}, {
		"17m",
		18068000,
		18118000,
		18168000
	}, {
		"15m",
		21000000,
		21225000,
		21450000
	}, {
		"12m",
		24890000,
		24940000,
		24990000
	}, {
		"10m",
		28000000,
		28850000,
		29700000
	}, {
		"6m",
		50000000,
		52000000,
		54000000
	}, {
		"2m",
		144000000,
		146000000,
		148000000
	}, {
		"1.25m",
		219000000,
		222000000,
		225000000
	}, {
		"70cm",
		420000000,
		435000000,
		450000000
	}, {
		"33cm",
		902000000,
		915000000,
		928000000
	}, {
		"23cm",
		1240000000,
		1270000000,
		1300000000
	}, {
		"13cmL",
		2300000000,
		2305000000,
		2310000000
	}, {
		"13cmU",
		2390000000,
		2420000000,
		2450000000
	}
};

/*! \brief Demodulator */
typedef struct {
	char demod_name[5];
	int8_t demod_type;
} demodulator_type;

/*! \brief Demodulators */
const demodulator_type demodulator[DEMOD_DEFINITIONS_QTY] = {
	{
		"None",
		0
	}, {
		"AM",
		1
	}, {
		"SAM",
		2
	}, {
		"NFM",
		3
	}, {
		"MFM",
		4
	}, {
		"WFM",
		5
	}, {
		"SWFM",
		6
	}, {
		"DSB",
		7
	}, {
		"LSB",
		8
	}, {
		"USB",
		9
	}, {
		"CW",
		10
	}, {
		"DIGI",
		11
	}, {
		"DAB",
		12
	}, {
		"IQ",
		13
	}
};


typedef struct {
	unsigned st_changed:1;
	unsigned upd_disp_all:1;
	unsigned upd_disp_sql:1;
	unsigned upd_disp_mode:1;
	unsigned upd_disp_vol:1;
	unsigned upd_disp_band:1;
	unsigned upd_disp_att:1;
	unsigned upd_disp_preamp:1;
} state_flags_t;


#endif	/* !REMOTE_H_ */
