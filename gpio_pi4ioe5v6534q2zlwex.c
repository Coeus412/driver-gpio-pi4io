// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for the Pericom PI4IOE5V6534Q2ZLWEX GPIO Expander.
 * Driver for the Pericom pi4ioe5v6534q2zlwex GPIO Expander.
 *
 * Copyright (C) 2024 Sy Phan
 * 
 * This driver is derived from gpio-pca953x.c
 * authored by Ben Gardner <bgardner@wabtec.com>
 * 
 * This driver is derived from gpio-pi4ioe5v6408.c
 * authored by Alex Van Damme
 * 
 */

#include <linux/acpi.h>
#include <linux/bitmap.h>
#include <linux/gpio/driver.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>
#include <linux/slab.h>

#include <asm/unaligned.h>

#define CONFIG_GPIO_PI4IO_IRQ

#define PI4IO_INPUT		            0x00
#define PI4IO_OUTPUT		        0x05
#define PI4IO_POLARITY_INVERSION	0x0A
#define PI4IO_CONFIGURATION	        0x0F

#define PI4IO_OUT_STRENGTH	        0x30
#define PI4IO_INPUT_LATCH	        0x3A
#define PI4IO_PUPD_EN	            0x3F
#define PI4IO_PUPD_SEL	            0x44
#define PI4IO_INT_MASK	            0x49
#define PI4IO_INT_STAT	            0x4E
#define PI4IO_OUT_CONFIG	        0x53

#define PI4IO_INT_EDGE	            0x54
#define PI4IO_INT_CLR	            0x5E
#define PI4IO_IN_STATUS	            0x63
#define PI4IO_OUT_INDCONF	        0x68
#define PI4IO_SW_DEBOUNCE	        0x6D

#define REG_ADDR_MASK		        GENMASK(5, 0)
#define REG_ADDR_EXT		        BIT(6)
#define REG_ADDR_AI		            BIT(7)

#define PCA_GPIO_MASK		        GENMASK(7, 0)

#define PCAL_GPIO_MASK		        GENMASK(4, 0)
#define PCAL_PINCTRL_MASK	        GENMASK(6, 5)

#define PCA_INT			            BIT(8)
#define PCA_PCAL		            BIT(9)
#define PCA_LATCH_INT       (PCA_PCAL | PCA_INT)
#define PI4IO_TYPE		            BIT(12)
#define PCA_TYPE_MASK		        GENMASK(15, 12)

#define PCA_CHIP_TYPE(x)	((x) & PCA_TYPE_MASK)

static const struct i2c_device_id pi4io_id[] = {
	{ "sppi4io", 34 | PI4IO_TYPE | PCA_LATCH_INT, },
	{ }
};
MODULE_DEVICE_TABLE(i2c, pi4io_id);

#ifdef CONFIG_GPIO_PI4IO_IRQ

#include <linux/dmi.h>

static const struct acpi_gpio_params pi4io_irq_gpios = { 0, 0, true };

static const struct acpi_gpio_mapping pi4io_acpi_irq_gpios[] = {
	{ "irq-gpios", &pi4io_irq_gpios, 1, ACPI_GPIO_QUIRK_ABSOLUTE_NUMBER },
	{ }
};

static int pi4io_acpi_get_irq(struct device *dev)
{
	int ret;

	ret = devm_acpi_dev_add_driver_gpios(dev, pi4io_acpi_irq_gpios);
	if (ret)
		dev_warn(dev, "can't add GPIO ACPI mapping\n");

	ret = acpi_dev_gpio_irq_get_by(ACPI_COMPANION(dev), "irq-gpios", 0);
	if (ret < 0)
		return ret;

	dev_info(dev, "ACPI interrupt quirk (IRQ %d)\n", ret);
	return ret;
}

static const struct dmi_system_id pi4io_dmi_acpi_irq_info[] = {
	{
		/*
		 * On Intel Galileo Gen 2 board the IRQ pin of one of
		 * the I²C GPIO expanders, which has GpioInt() resource,
		 * is provided as an absolute number instead of being
		 * relative. Since first controller (gpio-sch.c) and
		 * second (gpio-dwapb.c) are at the fixed bases, we may
		 * safely refer to the number in the global space to get
		 * an IRQ out of it.
		 */
		.matches = {
			DMI_EXACT_MATCH(DMI_BOARD_NAME, "GalileoGen2"),
		},
	},
	{}
};
#endif

static const struct acpi_device_id pi4io_acpi_ids[] = {
	{ "INT3491", 34 | PI4IO_TYPE | PCA_LATCH_INT, },
	{ }
};
MODULE_DEVICE_TABLE(acpi, pi4io_acpi_ids);

#include <linux/types.h>
#include <linux/i2c.h>

/* platform data for the PI4IO I/O expander driver */

struct pi4io_platform_data {
	/* number of the first GPIO */
	unsigned	gpio_base;

	/* initial polarity inversion setting */
	u32		invert;

	/* interrupt base */
	int		irq_base;

	void	*context;	/* param to setup/teardown */

	int		(*setup)(struct i2c_client *client,
				unsigned gpio, unsigned ngpio,
				void *context);
	int		(*teardown)(struct i2c_client *client,
				unsigned gpio, unsigned ngpio,
				void *context);
	const char	*const *names;
};

#define MAX_BANK 5
#define BANK_SZ 8
#define MAX_LINE	(MAX_BANK * BANK_SZ)

#define NBANK(chip) DIV_ROUND_UP(chip->gpio_chip.ngpio, BANK_SZ)

struct pi4io_reg_config {
	int direction;
	int output;
	int input;
	int invert;
};

static const struct pi4io_reg_config pi4io_regs = {
	.direction = PI4IO_CONFIGURATION,
	.output = PI4IO_OUTPUT,
	.input = PI4IO_INPUT,
	.invert = PI4IO_POLARITY_INVERSION,
};

struct pi4io_chip {
	unsigned gpio_start;
	struct mutex i2c_lock;
	struct regmap *regmap;

#ifdef CONFIG_GPIO_PI4IO_IRQ
	struct mutex irq_lock;
	DECLARE_BITMAP(irq_mask, MAX_LINE);
	DECLARE_BITMAP(irq_stat, MAX_LINE);
	DECLARE_BITMAP(irq_trig_raise, MAX_LINE);
	DECLARE_BITMAP(irq_trig_fall, MAX_LINE);
	struct irq_chip irq_chip;
#endif
	atomic_t wakeup_path;

	struct i2c_client *client;
	struct gpio_chip gpio_chip;
	const char *const *names;
	unsigned long driver_data;
	struct regulator *regulator;

	const struct pi4io_reg_config *regs;
};

static int pi4io_bank_shift(struct pi4io_chip *chip)
{
	return fls((chip->gpio_chip.ngpio - 1) / BANK_SZ);
}

#define PI4IO_BANK_INPUT	BIT(0)
#define PI4IO_BANK_OUTPUT	BIT(1)
#define PI4IO_BANK_POLARITY	BIT(2)
#define PI4IO_BANK_CONFIG	BIT(3)

#define PI4IO_BANK_IN_LATCH	BIT(8 + 2)
#define PI4IO_BANK_PULL_EN	BIT(8 + 3)
#define PI4IO_BANK_PULL_SEL	BIT(8 + 4)
#define PI4IO_BANK_IRQ_MASK	BIT(8 + 5)
#define PI4IO_BANK_IRQ_STAT	BIT(8 + 6)



static bool pi4io_check_register(struct pi4io_chip *chip, unsigned int reg,
				   u32 checkbank)
{
	int bank_shift = pi4io_bank_shift(chip);
	int bank = (reg & REG_ADDR_MASK) >> bank_shift;
	int offset = reg & (BIT(bank_shift) - 1);

	return true;
}

static bool pi4io_readable_register(struct device *dev, unsigned int reg)
{
	struct pi4io_chip *chip = dev_get_drvdata(dev);
	u32 bank;

	if (PCA_CHIP_TYPE(chip->driver_data) == PI4IO_TYPE) {
		bank = PI4IO_BANK_INPUT | PI4IO_BANK_OUTPUT |
		       PI4IO_BANK_POLARITY | PI4IO_BANK_CONFIG;
	}

	if (chip->driver_data & PCA_PCAL) {
		bank |= PI4IO_BANK_IN_LATCH | PI4IO_BANK_PULL_EN |
			PI4IO_BANK_PULL_SEL | PI4IO_BANK_IRQ_MASK |
			PI4IO_BANK_IRQ_STAT;
	}

	return pi4io_check_register(chip, reg, bank);
}

static bool pi4io_writeable_register(struct device *dev, unsigned int reg)
{
	struct pi4io_chip *chip = dev_get_drvdata(dev);
	u32 bank;

	if (PCA_CHIP_TYPE(chip->driver_data) == PI4IO_TYPE) {
		bank = PI4IO_BANK_OUTPUT | PI4IO_BANK_POLARITY |
			PI4IO_BANK_CONFIG;
	}

	if (chip->driver_data & PCA_PCAL)
		bank |= PI4IO_BANK_IN_LATCH | PI4IO_BANK_PULL_EN |
			PI4IO_BANK_PULL_SEL | PI4IO_BANK_IRQ_MASK;

	return pi4io_check_register(chip, reg, bank);
}

static bool pi4io_volatile_register(struct device *dev, unsigned int reg)
{
	struct pi4io_chip *chip = dev_get_drvdata(dev);
	u32 bank;

	if (PCA_CHIP_TYPE(chip->driver_data) == PI4IO_TYPE)
		bank = PI4IO_BANK_INPUT;

	if (chip->driver_data & PCA_PCAL)
		bank |= PI4IO_BANK_IRQ_STAT;

	return pi4io_check_register(chip, reg, bank);
}

static const struct regmap_config pi4io_i2c_regmap = {
	.reg_bits = 8,
	.val_bits = 8,

	.use_single_read = true,
	.use_single_write = true,

	.readable_reg = pi4io_readable_register,
	.writeable_reg = pi4io_writeable_register,
	.volatile_reg = pi4io_volatile_register,

	.disable_locking = true,
	.cache_type = REGCACHE_RBTREE,
	.max_register = 0x7f,
};

static const struct regmap_config pi4io_ai_i2c_regmap = {
	.reg_bits = 8,
	.val_bits = 8,

	.read_flag_mask = REG_ADDR_AI,
	.write_flag_mask = REG_ADDR_AI,

	.readable_reg = pi4io_readable_register,
	.writeable_reg = pi4io_writeable_register,
	.volatile_reg = pi4io_volatile_register,

	.disable_locking = true,
	.cache_type = REGCACHE_RBTREE,
	.max_register = 0x7f,
};

static u8 pi4io_recalc_addr(struct pi4io_chip *chip, int reg, int off)
{
	int bank_shift = pi4io_bank_shift(chip);
	int addr = (reg & PCAL_GPIO_MASK) << bank_shift;
	int pinctrl = (reg & PCAL_PINCTRL_MASK) << 1;
	int page = off / 8;
	u8 regaddr = reg + page;

	return regaddr;
}

static int pi4io_write_regs(struct pi4io_chip *chip, int reg, unsigned long *val)
{
	u8 regaddr = pi4io_recalc_addr(chip, reg, 0);
	u8 value[MAX_BANK];
	int i, ret;

	for (i = 0; i < NBANK(chip); i++)
		value[i] = bitmap_get_value8(val, i * BANK_SZ);

	ret = regmap_bulk_write(chip->regmap, regaddr, value, NBANK(chip));
	if (ret < 0) {
		dev_err(&chip->client->dev, "failed writing register\n");
		return ret;
	}

	return 0;
}

static int pi4io_read_regs(struct pi4io_chip *chip, int reg, unsigned long *val)
{
	u8 regaddr = pi4io_recalc_addr(chip, reg, 0);
	u8 value[MAX_BANK];
	int i, ret;

	ret = regmap_bulk_read(chip->regmap, regaddr, value, NBANK(chip));
	if (ret < 0) {
		dev_err(&chip->client->dev, "failed reading register\n");
		return ret;
	}

	for (i = 0; i < NBANK(chip); i++)
		bitmap_set_value8(val, value[i], i * BANK_SZ);

	return 0;
}

static int pi4io_gpio_direction_input(struct gpio_chip *gc, unsigned off)
{
	struct pi4io_chip *chip = gpiochip_get_data(gc);
	u8 dirreg = pi4io_recalc_addr(chip, chip->regs->direction, off);
	u8 bit = BIT(off % BANK_SZ);
	int ret;

	mutex_lock(&chip->i2c_lock);
	ret = regmap_write_bits(chip->regmap, dirreg, bit, bit);
	mutex_unlock(&chip->i2c_lock);
	return ret;
}

static int pi4io_gpio_direction_output(struct gpio_chip *gc,
		unsigned off, int val)
{
	struct pi4io_chip *chip = gpiochip_get_data(gc);
	u8 dirreg = pi4io_recalc_addr(chip, chip->regs->direction, off);
	u8 outreg = pi4io_recalc_addr(chip, chip->regs->output, off);
	u8 bit = BIT(off % BANK_SZ);
	int ret;

	mutex_lock(&chip->i2c_lock);
	/* set output level */
	ret = regmap_write_bits(chip->regmap, outreg, bit, val ? bit : 0);
	if (ret)
		goto exit;

	/* then direction */
	ret = regmap_write_bits(chip->regmap, dirreg, bit, 0);
exit:
	mutex_unlock(&chip->i2c_lock);
	return ret;
}

static int pi4io_gpio_get_value(struct gpio_chip *gc, unsigned off)
{
	struct pi4io_chip *chip = gpiochip_get_data(gc);
	u8 inreg = pi4io_recalc_addr(chip, chip->regs->input, off);
	u8 bit = BIT(off % BANK_SZ);
	u32 reg_val;
	int ret;

	mutex_lock(&chip->i2c_lock);
	ret = regmap_read(chip->regmap, inreg, &reg_val);
	mutex_unlock(&chip->i2c_lock);
	if (ret < 0)
		return ret;

	return !!(reg_val & bit);
}

static void pi4io_gpio_set_value(struct gpio_chip *gc, unsigned off, int val)
{
	struct pi4io_chip *chip = gpiochip_get_data(gc);
	u8 outreg = pi4io_recalc_addr(chip, chip->regs->output, off);
	u8 bit = BIT(off % BANK_SZ);

	mutex_lock(&chip->i2c_lock);
	regmap_write_bits(chip->regmap, outreg, bit, val ? bit : 0);
	mutex_unlock(&chip->i2c_lock);
}

static int pi4io_gpio_get_direction(struct gpio_chip *gc, unsigned off)
{
	struct pi4io_chip *chip = gpiochip_get_data(gc);
	u8 dirreg = pi4io_recalc_addr(chip, chip->regs->direction, off);
	u8 bit = BIT(off % BANK_SZ);
	u32 reg_val;
	int ret;

	mutex_lock(&chip->i2c_lock);
	ret = regmap_read(chip->regmap, dirreg, &reg_val);
	mutex_unlock(&chip->i2c_lock);
	if (ret < 0)
		return ret;

	if (reg_val & bit)
		return GPIO_LINE_DIRECTION_IN;

	return GPIO_LINE_DIRECTION_OUT;
}

static int pi4io_gpio_get_multiple(struct gpio_chip *gc,
				     unsigned long *mask, unsigned long *bits)
{
	struct pi4io_chip *chip = gpiochip_get_data(gc);
	DECLARE_BITMAP(reg_val, MAX_LINE);
	int ret;

	mutex_lock(&chip->i2c_lock);
	ret = pi4io_read_regs(chip, chip->regs->input, reg_val);
	mutex_unlock(&chip->i2c_lock);
	if (ret)
		return ret;

	bitmap_replace(bits, bits, reg_val, mask, gc->ngpio);
	return 0;
}

static void pi4io_gpio_set_multiple(struct gpio_chip *gc,
				      unsigned long *mask, unsigned long *bits)
{
	struct pi4io_chip *chip = gpiochip_get_data(gc);
	DECLARE_BITMAP(reg_val, MAX_LINE);
	int ret;

	mutex_lock(&chip->i2c_lock);
	ret = pi4io_read_regs(chip, chip->regs->output, reg_val);
	if (ret)
		goto exit;

	bitmap_replace(reg_val, reg_val, bits, mask, gc->ngpio);

	pi4io_write_regs(chip, chip->regs->output, reg_val);
exit:
	mutex_unlock(&chip->i2c_lock);
}

static int pi4io_gpio_set_pull_up_down(struct pi4io_chip *chip,
					 unsigned int offset,
					 unsigned long config)
{
	u8 pull_en_reg = pi4io_recalc_addr(chip, PI4IO_PUPD_EN, offset);
	u8 pull_sel_reg = pi4io_recalc_addr(chip, PI4IO_PUPD_SEL, offset);
	u8 bit = BIT(offset % BANK_SZ);
	int ret;

	/*
	 * pull-up/pull-down configuration requires PCAL extended
	 * registers
	 */
	if (!(chip->driver_data & PCA_PCAL))
		return -ENOTSUPP;

	mutex_lock(&chip->i2c_lock);

	/* Configure pull-up/pull-down */
	if (config == PIN_CONFIG_BIAS_PULL_UP)
		ret = regmap_write_bits(chip->regmap, pull_sel_reg, bit, bit);
	else if (config == PIN_CONFIG_BIAS_PULL_DOWN)
		ret = regmap_write_bits(chip->regmap, pull_sel_reg, bit, 0);
	else
		ret = 0;
	if (ret)
		goto exit;

	/* Disable/Enable pull-up/pull-down */
	if (config == PIN_CONFIG_BIAS_DISABLE)
		ret = regmap_write_bits(chip->regmap, pull_en_reg, bit, 0);
	else
		ret = regmap_write_bits(chip->regmap, pull_en_reg, bit, bit);

exit:
	mutex_unlock(&chip->i2c_lock);
	return ret;
}

static int pi4io_gpio_set_config(struct gpio_chip *gc, unsigned int offset,
				   unsigned long config)
{
	struct pi4io_chip *chip = gpiochip_get_data(gc);

	switch (pinconf_to_config_param(config)) {
	case PIN_CONFIG_BIAS_PULL_UP:
	case PIN_CONFIG_BIAS_PULL_PIN_DEFAULT:
	case PIN_CONFIG_BIAS_PULL_DOWN:
	case PIN_CONFIG_BIAS_DISABLE:
		return pi4io_gpio_set_pull_up_down(chip, offset, config);
	default:
		return -ENOTSUPP;
	}
}

static void pi4io_setup_gpio(struct pi4io_chip *chip, int gpios)
{
	struct gpio_chip *gc;

	gc = &chip->gpio_chip;

	gc->direction_input  = pi4io_gpio_direction_input;
	gc->direction_output = pi4io_gpio_direction_output;
	gc->get = pi4io_gpio_get_value;
	gc->set = pi4io_gpio_set_value;
	gc->get_direction = pi4io_gpio_get_direction;
	gc->get_multiple = pi4io_gpio_get_multiple;
	gc->set_multiple = pi4io_gpio_set_multiple;
	gc->set_config = pi4io_gpio_set_config;
	gc->can_sleep = true;

	gc->base = chip->gpio_start;
	gc->ngpio = gpios;
	gc->label = dev_name(&chip->client->dev);
	gc->parent = &chip->client->dev;
	gc->owner = THIS_MODULE;
	gc->names = chip->names;
}

#ifdef CONFIG_GPIO_PI4IO_IRQ
static void pi4io_irq_mask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct pi4io_chip *chip = gpiochip_get_data(gc);
	irq_hw_number_t hwirq = irqd_to_hwirq(d);

	clear_bit(hwirq, chip->irq_mask);
}

static void pi4io_irq_unmask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct pi4io_chip *chip = gpiochip_get_data(gc);
	irq_hw_number_t hwirq = irqd_to_hwirq(d);

	set_bit(hwirq, chip->irq_mask);
}

static int pi4io_irq_set_wake(struct irq_data *d, unsigned int on)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct pi4io_chip *chip = gpiochip_get_data(gc);

	if (on)
		atomic_inc(&chip->wakeup_path);
	else
		atomic_dec(&chip->wakeup_path);

	return irq_set_irq_wake(chip->client->irq, on);
}

static void pi4io_irq_bus_lock(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct pi4io_chip *chip = gpiochip_get_data(gc);

	mutex_lock(&chip->irq_lock);
}

static void pi4io_irq_bus_sync_unlock(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct pi4io_chip *chip = gpiochip_get_data(gc);
	DECLARE_BITMAP(irq_mask, MAX_LINE);
	DECLARE_BITMAP(reg_direction, MAX_LINE);
	int level;

	if (chip->driver_data & PCA_PCAL) {
		/* Enable latch on interrupt-enabled inputs */
		pi4io_write_regs(chip, PI4IO_INPUT_LATCH, chip->irq_mask);

		bitmap_complement(irq_mask, chip->irq_mask, gc->ngpio);

		/* Unmask enabled interrupts */
		pi4io_write_regs(chip, PI4IO_INT_MASK, irq_mask);
	}

	/* Switch direction to input if needed */
	pi4io_read_regs(chip, chip->regs->direction, reg_direction);

	bitmap_or(irq_mask, chip->irq_trig_fall, chip->irq_trig_raise, gc->ngpio);
	bitmap_complement(reg_direction, reg_direction, gc->ngpio);
	bitmap_and(irq_mask, irq_mask, reg_direction, gc->ngpio);

	/* Look for any newly setup interrupt */
	for_each_set_bit(level, irq_mask, gc->ngpio)
		pi4io_gpio_direction_input(&chip->gpio_chip, level);

	mutex_unlock(&chip->irq_lock);
}

static int pi4io_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct pi4io_chip *chip = gpiochip_get_data(gc);
	irq_hw_number_t hwirq = irqd_to_hwirq(d);

	if (!(type & IRQ_TYPE_EDGE_BOTH)) {
		dev_err(&chip->client->dev, "irq %d: unsupported type %d\n",
			d->irq, type);
		return -EINVAL;
	}

	assign_bit(hwirq, chip->irq_trig_fall, type & IRQ_TYPE_EDGE_FALLING);
	assign_bit(hwirq, chip->irq_trig_raise, type & IRQ_TYPE_EDGE_RISING);

	return 0;
}

static void pi4io_irq_shutdown(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct pi4io_chip *chip = gpiochip_get_data(gc);
	irq_hw_number_t hwirq = irqd_to_hwirq(d);

	clear_bit(hwirq, chip->irq_trig_raise);
	clear_bit(hwirq, chip->irq_trig_fall);
}

static bool pi4io_irq_pending(struct pi4io_chip *chip, unsigned long *pending)
{
	struct gpio_chip *gc = &chip->gpio_chip;
	DECLARE_BITMAP(reg_direction, MAX_LINE);
	DECLARE_BITMAP(old_stat, MAX_LINE);
	DECLARE_BITMAP(cur_stat, MAX_LINE);
	DECLARE_BITMAP(new_stat, MAX_LINE);
	DECLARE_BITMAP(trigger, MAX_LINE);
	int ret;

	if (chip->driver_data & PCA_PCAL) {
		/* Read the current interrupt status from the device */
		ret = pi4io_read_regs(chip, PI4IO_INT_STAT, trigger);
		if (ret)
			return false;

		/* Check latched inputs and clear interrupt status */
		ret = pi4io_read_regs(chip, chip->regs->input, cur_stat);
		if (ret)
			return false;

		/* Apply filter for rising/falling edge selection */
		bitmap_replace(new_stat, chip->irq_trig_fall, chip->irq_trig_raise, cur_stat, gc->ngpio);

		bitmap_and(pending, new_stat, trigger, gc->ngpio);

		return !bitmap_empty(pending, gc->ngpio);
	}

	ret = pi4io_read_regs(chip, chip->regs->input, cur_stat);
	if (ret)
		return false;

	/* Remove output pins from the equation */
	pi4io_read_regs(chip, chip->regs->direction, reg_direction);

	bitmap_copy(old_stat, chip->irq_stat, gc->ngpio);

	bitmap_and(new_stat, cur_stat, reg_direction, gc->ngpio);
	bitmap_xor(cur_stat, new_stat, old_stat, gc->ngpio);
	bitmap_and(trigger, cur_stat, chip->irq_mask, gc->ngpio);

	bitmap_copy(chip->irq_stat, new_stat, gc->ngpio);

	if (bitmap_empty(trigger, gc->ngpio))
		return false;

	bitmap_and(cur_stat, chip->irq_trig_fall, old_stat, gc->ngpio);
	bitmap_and(old_stat, chip->irq_trig_raise, new_stat, gc->ngpio);
	bitmap_or(new_stat, old_stat, cur_stat, gc->ngpio);
	bitmap_and(pending, new_stat, trigger, gc->ngpio);

	return !bitmap_empty(pending, gc->ngpio);
}

static irqreturn_t pi4io_irq_handler(int irq, void *devid)
{
	struct pi4io_chip *chip = devid;
	struct gpio_chip *gc = &chip->gpio_chip;
	DECLARE_BITMAP(pending, MAX_LINE);
	int level;
	bool ret;

	bitmap_zero(pending, MAX_LINE);

	mutex_lock(&chip->i2c_lock);
	ret = pi4io_irq_pending(chip, pending);
	mutex_unlock(&chip->i2c_lock);

	if (ret) {
		ret = 0;

		for_each_set_bit(level, pending, gc->ngpio) {
			int nested_irq = irq_find_mapping(gc->irq.domain, level);

			if (unlikely(nested_irq <= 0)) {
				dev_warn_ratelimited(gc->parent, "unmapped interrupt %d\n", level);
				continue;
			}

			handle_nested_irq(nested_irq);
			ret = 1;
		}
	}

	return IRQ_RETVAL(ret);
}

static int pi4io_irq_setup(struct pi4io_chip *chip, int irq_base)
{
	struct i2c_client *client = chip->client;
	struct irq_chip *irq_chip = &chip->irq_chip;
	DECLARE_BITMAP(reg_direction, MAX_LINE);
	DECLARE_BITMAP(irq_stat, MAX_LINE);
	struct gpio_irq_chip *girq;
	int ret;

	if (dmi_first_match(pi4io_dmi_acpi_irq_info)) {
		ret = pi4io_acpi_get_irq(&client->dev);
		if (ret > 0)
			client->irq = ret;
	}

	if (!client->irq)
		return 0;

	if (irq_base == -1)
		return 0;

	if (!(chip->driver_data & PCA_INT))
		return 0;

	ret = pi4io_read_regs(chip, chip->regs->input, irq_stat);
	if (ret)
		return ret;

	/*
	 * There is no way to know which GPIO line generated the
	 * interrupt.  We have to rely on the previous read for
	 * this purpose.
	 */
	pi4io_read_regs(chip, chip->regs->direction, reg_direction);
	bitmap_and(chip->irq_stat, irq_stat, reg_direction, chip->gpio_chip.ngpio);
	mutex_init(&chip->irq_lock);

	irq_chip->name = dev_name(&client->dev);
	irq_chip->irq_mask = pi4io_irq_mask;
	irq_chip->irq_unmask = pi4io_irq_unmask;
	irq_chip->irq_set_wake = pi4io_irq_set_wake;
	irq_chip->irq_bus_lock = pi4io_irq_bus_lock;
	irq_chip->irq_bus_sync_unlock = pi4io_irq_bus_sync_unlock;
	irq_chip->irq_set_type = pi4io_irq_set_type;
	irq_chip->irq_shutdown = pi4io_irq_shutdown;

	girq = &chip->gpio_chip.irq;
	girq->chip = irq_chip;
	/* This will let us handle the parent IRQ in the driver */
	girq->parent_handler = NULL;
	girq->num_parents = 0;
	girq->parents = NULL;
	girq->default_type = IRQ_TYPE_NONE;
	girq->handler = handle_simple_irq;
	girq->threaded = true;
	girq->first = irq_base; /* FIXME: get rid of this */

	ret = devm_request_threaded_irq(&client->dev, client->irq,
					NULL, pi4io_irq_handler,
					IRQF_ONESHOT | IRQF_SHARED,
					dev_name(&client->dev), chip);
	if (ret) {
		dev_err(&client->dev, "failed to request irq %d\n",
			client->irq);
		return ret;
	}

	return 0;
}

#else /* CONFIG_GPIO_PI4IO_IRQ */
static int pi4io_irq_setup(struct pi4io_chip *chip,
			     int irq_base)
{
	struct i2c_client *client = chip->client;

	if (client->irq && irq_base != -1 && (chip->driver_data & PCA_INT))
		dev_warn(&client->dev, "interrupt support not compiled in\n");

	return 0;
}
#endif

static int device_pi4io_init(struct pi4io_chip *chip, u32 invert)
{
	DECLARE_BITMAP(val, MAX_LINE);
	int ret;
	u8 regaddr;

	regaddr = pi4io_recalc_addr(chip, chip->regs->output, 0);
	ret = regcache_sync_region(chip->regmap, regaddr,
				   regaddr + NBANK(chip) - 1);
	if (ret)
		goto out;

	regaddr = pi4io_recalc_addr(chip, chip->regs->direction, 0);
	ret = regcache_sync_region(chip->regmap, regaddr,
				   regaddr + NBANK(chip) - 1);
	if (ret)
		goto out;

	/* set platform specific polarity inversion */
	if (invert)
		bitmap_fill(val, MAX_LINE);
	else
		bitmap_zero(val, MAX_LINE);

	ret = pi4io_write_regs(chip, chip->regs->invert, val);
out:
	return ret;
}

static int pi4io_probe(struct i2c_client *client,
			 const struct i2c_device_id *i2c_id)
{
	printk("PI4IO Driver is Probed!");

	struct pi4io_platform_data *pdata;
	struct pi4io_chip *chip;
	int irq_base = 0;
	int ret;
	u32 invert = 0;
	struct regulator *reg;
	const struct regmap_config *regmap_config;

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (chip == NULL)
		return -ENOMEM;

	pdata = dev_get_platdata(&client->dev);
	if (pdata) {
		irq_base = pdata->irq_base;
		chip->gpio_start = pdata->gpio_base;
		invert = pdata->invert;
		chip->names = pdata->names;
	} else {
		struct gpio_desc *reset_gpio;

		chip->gpio_start = -1;
		irq_base = 0;

		/*
		 * See if we need to de-assert a reset pin.
		 *
		 * There is no known ACPI-enabled platforms that are
		 * using "reset" GPIO. Otherwise any of those platform
		 * must use _DSD method with corresponding property.
		 */
		reset_gpio = devm_gpiod_get_optional(&client->dev, "reset",
						     GPIOD_OUT_LOW);
		if (IS_ERR(reset_gpio))
			return PTR_ERR(reset_gpio);
	}

	chip->client = client;

	reg = devm_regulator_get(&client->dev, "vcc");
	if (IS_ERR(reg))
		return dev_err_probe(&client->dev, PTR_ERR(reg), "reg get err\n");

	ret = regulator_enable(reg);
	if (ret) {
		dev_err(&client->dev, "reg en err: %d\n", ret);
		return ret;
	}
	chip->regulator = reg;

	if (i2c_id) {
		chip->driver_data = i2c_id->driver_data;
	} else {
		const void *match;

		match = device_get_match_data(&client->dev);
		if (!match) {
			ret = -ENODEV;
			goto err_exit;
		}

		chip->driver_data = (uintptr_t)match;
	}

	i2c_set_clientdata(client, chip);

	pi4io_setup_gpio(chip, chip->driver_data & PCA_GPIO_MASK);

	regmap_config = &pi4io_i2c_regmap;

	chip->regmap = devm_regmap_init_i2c(client, regmap_config);
	if (IS_ERR(chip->regmap)) {
		ret = PTR_ERR(chip->regmap);
		goto err_exit;
	}

	regcache_mark_dirty(chip->regmap);

	mutex_init(&chip->i2c_lock);

	lockdep_set_subclass(&chip->i2c_lock,
			     i2c_adapter_depth(client->adapter));

	ret = device_reset(&client->dev);
	if (ret == -EPROBE_DEFER)
		return -EPROBE_DEFER;

	/* initialize cached registers from their original values.
	 * we can't share this chip with another i2c master.
	 */

	if (PCA_CHIP_TYPE(chip->driver_data) == PI4IO_TYPE) {
		chip->regs = &pi4io_regs;
		ret = device_pi4io_init(chip, invert);
	}

	if (ret)
		goto err_exit;

	ret = pi4io_irq_setup(chip, irq_base);
	if (ret)
		goto err_exit;

	ret = devm_gpiochip_add_data(&client->dev, &chip->gpio_chip, chip);
	if (ret)
		goto err_exit;

	if (pdata && pdata->setup) {
		ret = pdata->setup(client, chip->gpio_chip.base,
				   chip->gpio_chip.ngpio, pdata->context);
		if (ret < 0)
			dev_warn(&client->dev, "setup failed, %d\n", ret);
	}

	return 0;

err_exit:
	regulator_disable(chip->regulator);
	return ret;
}

static int pi4io_remove(struct i2c_client *client)
{
	struct pi4io_platform_data *pdata = dev_get_platdata(&client->dev);
	struct pi4io_chip *chip = i2c_get_clientdata(client);
	int ret;

	if (pdata && pdata->teardown) {
		ret = pdata->teardown(client, chip->gpio_chip.base,
				      chip->gpio_chip.ngpio, pdata->context);
		if (ret < 0)
			dev_err(&client->dev, "teardown failed, %d\n", ret);
	} else {
		ret = 0;
	}

	regulator_disable(chip->regulator);

	return ret;
}

#ifdef CONFIG_PM_SLEEP
static int pi4io_regcache_sync(struct device *dev)
{
	struct pi4io_chip *chip = dev_get_drvdata(dev);
	int ret;
	u8 regaddr;

	/*
	 * The ordering between direction and output is important,
	 * sync these registers first and only then sync the rest.
	 */
	regaddr = pi4io_recalc_addr(chip, chip->regs->direction, 0);
	ret = regcache_sync_region(chip->regmap, regaddr, regaddr + NBANK(chip) - 1);
	if (ret) {
		dev_err(dev, "Failed to sync GPIO dir registers: %d\n", ret);
		return ret;
	}

	regaddr = pi4io_recalc_addr(chip, chip->regs->output, 0);
	ret = regcache_sync_region(chip->regmap, regaddr, regaddr + NBANK(chip) - 1);
	if (ret) {
		dev_err(dev, "Failed to sync GPIO out registers: %d\n", ret);
		return ret;
	}

#ifdef CONFIG_GPIO_PI4IO_IRQ
	if (chip->driver_data & PCA_PCAL) {
		regaddr = pi4io_recalc_addr(chip, PI4IO_INPUT_LATCH, 0);
		ret = regcache_sync_region(chip->regmap, regaddr,
					   regaddr + NBANK(chip) - 1);
		if (ret) {
			dev_err(dev, "Failed to sync INT latch registers: %d\n",
				ret);
			return ret;
		}

		regaddr = pi4io_recalc_addr(chip, PI4IO_INT_MASK, 0);
		ret = regcache_sync_region(chip->regmap, regaddr,
					   regaddr + NBANK(chip) - 1);
		if (ret) {
			dev_err(dev, "Failed to sync INT mask registers: %d\n",
				ret);
			return ret;
		}
	}
#endif

	return 0;
}

static int pi4io_suspend(struct device *dev)
{
	struct pi4io_chip *chip = dev_get_drvdata(dev);

	mutex_lock(&chip->i2c_lock);
	regcache_cache_only(chip->regmap, true);
	mutex_unlock(&chip->i2c_lock);

	if (atomic_read(&chip->wakeup_path))
		device_set_wakeup_path(dev);
	else
		regulator_disable(chip->regulator);

	return 0;
}

static int pi4io_resume(struct device *dev)
{
	struct pi4io_chip *chip = dev_get_drvdata(dev);
	int ret;

	if (!atomic_read(&chip->wakeup_path)) {
		ret = regulator_enable(chip->regulator);
		if (ret) {
			dev_err(dev, "Failed to enable regulator: %d\n", ret);
			return 0;
		}
	}

	mutex_lock(&chip->i2c_lock);
	regcache_cache_only(chip->regmap, false);
	regcache_mark_dirty(chip->regmap);
	ret = pi4io_regcache_sync(dev);
	if (ret) {
		mutex_unlock(&chip->i2c_lock);
		return ret;
	}

	ret = regcache_sync(chip->regmap);
	mutex_unlock(&chip->i2c_lock);
	if (ret) {
		dev_err(dev, "Failed to restore register map: %d\n", ret);
		return ret;
	}

	return 0;
}
#endif

/* convenience to stop overlong match-table lines */
#define OF_PI4IOX(__nrgpio, __int) (void *)(__nrgpio | PI4IO_TYPE | __int)

static const struct of_device_id pi4io_dt_ids[] = {
	{ .compatible = "syphan,pi4io", .data = OF_PI4IOX(16, PCA_LATCH_INT), },
	{ }
};

MODULE_DEVICE_TABLE(of, pi4io_dt_ids);

static SIMPLE_DEV_PM_OPS(pi4io_pm_ops, pi4io_suspend, pi4io_resume);

static struct i2c_driver pi4io_driver = {
	.driver = {
		.name	= "sp-pi4io",
		.pm	= &pi4io_pm_ops,
		.of_match_table = pi4io_dt_ids,
		.acpi_match_table = pi4io_acpi_ids,
	},
	.probe		= pi4io_probe,
	.remove		= pi4io_remove,
	.id_table	= pi4io_id,
};

static int __init pi4io_init(void)
{
	return i2c_add_driver(&pi4io_driver);
}

subsys_initcall(pi4io_init);

static void __exit pi4io_exit(void)
{
	i2c_del_driver(&pi4io_driver);
}
module_exit(pi4io_exit);

MODULE_AUTHOR("Sy Phan");
MODULE_DESCRIPTION("The Driver for PI4IOE5V6534Q2ZLWEX");
MODULE_LICENSE("GPL");
MODULE_VERSION("2:0.0");