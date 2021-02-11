import Resistance from "@/components/station/Resistance.vue";
import useVuetify from "@/hooks/useVuetify";
import { mount } from '@vue/test-utils'

const wrapper = useVuetify(Resistance);

describe("When mounting Resistance component", () => {
  it("Should mount", () => {
    expect(wrapper.vm).toBeTruthy();
  });
});

describe("When specify props to Resistance component", () => {
  const wrapper = mount(Resistance, {
      propsData: {
          resistanceValue: 100000,
          pucksColors: ['red', 'blue', 'yellow']
      }
    })

    it("Should be the right ones", () => {
          expect(wrapper.props().resistanceValue).toBe(100000)
          expect(wrapper.props().pucksColors).toStrictEqual(['red', 'blue', 'yellow'])
    });
  });

  describe("When mounting Resistance component with props", () => {
    const wrapper = mount(Resistance, {
      propsData: {
          resistanceValue: 100000,
          pucksColors: ['red', 'blue', 'yellow']
      }
    })

    it("Should contains resistanceValue", () => {
      const resistanceValue = wrapper.findComponent({ref: 'resistanceValue'}) 

      expect(resistanceValue.exists()).toBe(true)
      expect(resistanceValue.text()).toBe('100000 Ω')
    });

    it("Should contains pucks", () => {
      const pucks = wrapper.findAllComponents({ref: 'pucks'})

      expect(pucks.exists()).toBe(true)
      expect(pucks).toHaveLength(3)
    });
  });

  
  describe("When mounting Resistance component without props", () => {
    const wrapper = mount(Resistance,{})

    it("Should not contains resistanceValue", () => {
      const resistanceValue = wrapper.findComponent({ref: 'resistanceValue'}) 

      expect(resistanceValue.exists()).toBe(true)
      expect(resistanceValue.text()).toBe('Ω')
    });

    it("Should not contains pucks", () => {
      const pucks = wrapper.findAllComponents({ref: 'pucks'})

      expect(pucks.exists()).toBe(false)
      expect(pucks).toHaveLength(0)
    });
  });