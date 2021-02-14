import ControlPanel from '@/components/station/ControlPanel.vue';
import wrapWithVuetify from '@/util/wrapWithVuetify';
import { shallowMount } from '@vue/test-utils';
import { ControlPanelResultFactory } from '@/factories/ControlPanelResult';
import { ColorFactory } from '@/factories/ColorFactory';
import ControlPanelResult from '@/classes/ControlPanelResult';

const wrapper = wrapWithVuetify(ControlPanel);

describe('When mounting ControlPanel component', () => {
  it('Should mount', () => {
    expect(wrapper.vm).toBeTruthy();
  });
});

describe('Given props', () => {
    const controlPanelResultExpected = ControlPanelResultFactory.get() as ControlPanelResult;
    const colorFirstPuckExpected = ColorFactory.get();

  const props = {
    controlPanelResult: controlPanelResultExpected,
    colorFirstPuck: colorFirstPuckExpected,
  };
  describe('When mounting ControlPanel', () => {
    const wrapper = shallowMount(ControlPanel, {
      propsData: props,
    });

    it('Should be the right props', () => {
      expect(wrapper.props().controlPanelResult).toBe(controlPanelResultExpected);
      expect(wrapper.props().colorFirstPuck).toBe(colorFirstPuckExpected);
    });

    it('Should contains the right letter of corner', () => {
      const letterCorner = wrapper.findComponent({ ref: 'corner' });

      expect(letterCorner.exists()).toBe(true);
      expect(letterCorner.text()).toBe(controlPanelResultExpected.corner);
    });
  });
});

describe('Given no props', () => {
  describe('When mounting ControlPanel component without props', () => {
    const wrapper = shallowMount(ControlPanel, {});

    it('Should not contains resistanceValue', () => {
      const letterCorner = wrapper.findComponent({ ref: 'corner' });

      expect(letterCorner.exists()).toBe(true);
      expect(letterCorner.text()).toBe('');
    });
  });
});
