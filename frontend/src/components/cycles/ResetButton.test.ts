import wrapWithVuetifyAndStore from '@/util/wrapWithVuetifyAndStore';
import ResetButton from '@/components/cycles/ResetButton.vue';

describe('When mounting ResetButton component', () => {
  const wrapper = wrapWithVuetifyAndStore(ResetButton);

  it('Should mount', () => {
    expect(wrapper.vm).toBeTruthy();
  });
});
