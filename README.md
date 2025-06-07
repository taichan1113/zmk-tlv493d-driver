# ZMK Module Template

This repository contains a template for a ZMK module, as it would most frequently be used. 

## Usage

Read through the [ZMK Module Creation](https://zmk.dev/docs/development/module-creation) page for details on how to configure this template.

## More Info

For more info on modules, you can read through  through the [Zephyr modules page](https://docs.zephyrproject.org/3.5.0/develop/modules.html) and [ZMK's page on using modules](https://zmk.dev/docs/features/modules). [Zephyr's west manifest page](https://docs.zephyrproject.org/3.5.0/develop/west/manifest.html#west-manifests) may also be of use.


## memo

```dts
&i2c0 {
    tlv493d@5e {
        compatible = "infineon,tlv493d";
        reg = <0x5e>;
        label = "TLV493D";
    };
};
```

```c
#define TLV493D_NODE DT_NODELABEL(tlv493d)

const struct device *dev = DEVICE_DT_GET(TLV493D_NODE);
if (!device_is_ready(dev)) {
    printk("TLV493D device not ready\n");
}
```

```dts
&i2c0 {
    tlv493d@5e {
        compatible = "infineon,tlv493d";
        reg = <0x5e>;
        label = "TLV493D";
        irq-gpios = <&gpio0 10 GPIO_ACTIVE_LOW>;
    };
};
```
