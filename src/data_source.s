
    AREA _WAVData, DATA, READONLY

    EXPORT  WAVDataBegin
    EXPORT  WAVDataEnd

WAVDataBegin
	INCBIN .\US_P3_4_F.wav
WAVDataEnd

	END