class AudioInputBytes final : public PullAudioInputStreamCallback{
    public:
        // Constructor that creates an input stream from a file.
        AudioInputBytes(vector<uint8_t> midsample){
            this->midsample=midsample;
            position=0;
        }

        // Implements AudioInputStream::Read() which is called to get data from the audio stream.
        // It copies data available in the stream to 'dataBuffer', but no more than 'size' bytes.
        // If the data available is less than 'size' bytes, it is allowed to just return the amount of data that is currently available.
        // If there is no data, this function must wait until data is available.
        // It returns the number of bytes that have been copied in 'dataBuffer'.
        // It returns 0 to indicate that the stream reaches end or is closed.
        int Read(uint8_t* dataBuffer, uint32_t size) override{
            int bytesread=0;
            if (position==midsample.size())
                // returns 0 to indicate that the stream reaches end.
                return 0;
            
            //uint8_t arrayBuffer[size];
            
            for(int i=0;i<size;i++){dataBuffer[i]=0;}

            for(int i=position;(i-position)<size && i<midsample.size();i++){
                dataBuffer[i-position]=midsample[i];
                bytesread++;
            }
            // dataBuffer=arrayBuffer;
            position+=size;

            if (position>=midsample.size())
                // returns 0 to close the stream on read error.
                return 0;
            else
                // returns the number of bytes that have been read.
                return bytesread;
        }

        // Implements AudioInputStream::Close() which is called when the stream needs to be closed.
        void Close() override{
            
        }

    private:
        vector<uint8_t> midsample;
        int position=0;
};