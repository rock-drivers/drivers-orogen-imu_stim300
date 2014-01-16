require 'orocos'
require 'vizkit'
include Orocos

if !ARGV[0]
    STDERR.puts "usage: test.rb <device name>"
    exit 1
end

ENV['PKG_CONFIG_PATH'] = "#{File.expand_path("..", File.dirname(__FILE__))}/build:#{ENV['PKG_CONFIG_PATH']}"

Orocos.initialize

Orocos::Process.run 'stim300::Task' => 'stim300' do
    # log all the output ports
    Orocos.log_all_ports 
    Orocos.conf.load_dir('../config')

    # Get the task
    driver = Orocos.name_service.get 'stim300'
    Orocos.conf.apply(driver, ['default','Bremen','stim300_10g'], :override => true)

    driver.port = ARGV[0]
    driver.revision = 'REV_B'
    driver.timeout = 2
    driver.use_filter = true

    driver.configure
    driver.start

    # Orientation visualization
    attitudeRBS = Vizkit.default_loader.RigidBodyStateVisualization
    attitudeRBS.setPluginName("Attitude")
    attitudeRBS.loadModel("./stim300.stl")
    #attitudeRBS.setColor(Eigen::Vector3.new(255, 0, 0))#Red
    #attitudeRBS.resetModel(0.4)
    #Vizkit.vizkit3d_widget.setPluginDataFrame("world", attitudeRBS)

    Vizkit.display driver.port('orientation_samples_out'), :widget =>attitudeRBS


    ## Create a widget for 3d display
    view3d = Vizkit.vizkit3d_widget

    # Show it
    view3d.show

    Vizkit.exec

#     reader = driver.orientation_samples.reader(:type => :buffer, :size => 100)
#    loop do        
#  	while sample = reader.read_new
#            print("#{sample.time.to_f} #{sample.orientation.x} #{sample.orientation.y} #{sample.orientation.z} #{sample.orientation.w}\r\n")
#  	end
#	sleep 0.01
#    end

end
