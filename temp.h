class Controller_name{
    public:
        Controller_name();
        void init();
        void set_param();
        double cal_u();
        void reset();
        void merge();

        void set_param_1();
        void set_param_2();
        void get_param_1();
        void get_param_2();

    private:
        double param_1;
        double param_2;
};