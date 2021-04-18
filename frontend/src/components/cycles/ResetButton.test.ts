import wrapComponentForTest from '../../util/wrapComponentForTest';
import ResetButton from '@/components/cycles/ResetButton.vue';

describe('When mounting ResetButton component', () => {
  const wrapper = wrapComponentForTest(ResetButton);

  it('Should mount', () => {
    expect(wrapper.vm).toBeTruthy();
  });
});
