function [ emu ] = imc_init(full)
    if full
        javaaddpath('/usr/local/lib/libimc.jar');
        import('pt.lsts.imc.control.*');
    end

    emu = ControlLink.acquire('modem-emu', 30000)
end
