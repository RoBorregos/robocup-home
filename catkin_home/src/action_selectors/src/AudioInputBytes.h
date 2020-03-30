class AudioInputBytes final : public PullAudioInputStreamCallback{
    public:
        // Constructor that creates an input stream from a file.
        AudioInputBytes(vector<uint8_t> midsample){
            this->midsample=midsample;
            position=0;
        }

        // Implements AudioInputStream::Read() which is called to get data from the audio stream.
        int Read(uint8_t* dataBuffer, uint32_t size) override{
            
            int bytesread=0;
            
            // returns 0 to indicate that the stream reaches end.
            if (position==midsample.size())
                return 0;
            
            for(int i=0;i<size;i++){dataBuffer[i]=0;}

            for(int i=position;(i-position)<size && i<midsample.size();i++){
                dataBuffer[i-position]=midsample[i];
                bytesread++;
            }
            
            position+=size;

            // returns 0 to close the stream on read error.
            if (position>=midsample.size())
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