//#includes

class costmap{
    public:

    protected:
    float resolution; // meters/cell
    float x_orig;
    float y_orig;
    float x_size;
    float y_size;
    unsigned int cell_x_size;
    unsigned int cell_y_size;
    std::vector<std::vector<int8_t>> mat;

    private:
    int8_t default_cost;
}